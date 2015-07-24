/**
 ******************************************************************************
 * @file    IO_Toggle/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    19-September-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"

#include "usart2.h"
#include "ctrltbl.h"

#include <stdio.h>
#include <limits.h>
#include <math.h>


#define kPWMMAX (1000)	//PWMのフルスケール値
#define _PC10 ((uint16_t)(1<<10)) // LED1
#define _PC11 ((uint16_t)(1<<11)) // LED2
#define _PB10 ((uint16_t)(1<<10)) // USART3 TX
#define _PB11 ((uint16_t)(1<<11)) // USART3 RX
#define _PB2  ((uint16_t)(1<<2))  // USART3 DIR
#define _PB12 ((uint16_t)(1<<12)) // USART3 TERM

#define _PA8  ((uint16_t)(1<<8))
#define _PA7  ((uint16_t)(1<<7))

#define _PA9  ((uint16_t)(1<<9))
#define _PB14 ((uint16_t)(1<<14))

#define _PA10 ((uint16_t)(1<<10))
#define _PB15 ((uint16_t)(1<<15))

void Delay(__IO uint32_t nCount);
void xTim1Init();
void xSetPwm(const int16_t duty);
void xGPIOInit();
void xSpi2Init();
void xSpi3Init();
void xTim5Init();
void xTim4Init();
float xCurrentRead(void);
static unsigned char  crc8(const void* buff, size_t size);
void xADCInit();
void xLed(int sw);
void xTim2Init(void);
void xSetTim2CountDir(void);
void xTim3Init(void);
float xLpf(float in, int reset);
uint8_t xTdkRead8bit(uint16_t adr);
uint16_t xTdkRead16bit(uint16_t adr);
void xTdkWrite8bit(uint16_t adr,uint8_t data);
void xTdkWrite16bit(uint16_t adr, uint16_t writedata);
void xTdkAllRead(void);
void xTdkDebugMode(void);
void xEXTILineConfig(void);
void xCurrentSensorCalib(void);
void xTdkSetZero(void);

#define SPI2_CS_HIGH()	GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define SPI2_CS_LOW()	GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define SPI3_CS_HIGH()	GPIO_SetBits(GPIOD, GPIO_Pin_2)
#define SPI3_CS_LOW()	GPIO_ResetBits(GPIOD, GPIO_Pin_2)

TControlTable gT;
uint16_t g_ad_value[8];
int32_t gTimCount;
uint16_t g_motor_pos_pre=0;
uint16_t g_motor_pos_pre2=0;
float gPulse2Rad;
float gVolt2Duty;
long gLineInt;

#define kSamplingTime (0.0001)	//制御周期10KHz=0.0001[sec]
#define kSamplingFreq (10000.0)	//制御周波数=10KHz[Hz]
#define kPI (3.14159265)
#define kEncPulsex4 (4000.0)
#define kSpiReadMax 100

uint16_t g_move_mode=0;	//モータ駆動モード 0:直線 1:sinで往復
int main(void)
{

	xLpf(0,1);	//フィルタリセット

	xGPIOInit();
	xUSART2_init();
	xTim1Init();	//PWM
	xSetPwm(0);
	xSpi2Init();
	//xSpi3Init();
	xTim5Init();	//TIM5をエンコーダカウンタとして使う
	xTim4Init();	//TIM4をエンコーダカウンタとして使う
	xADCInit();
	xTim2Init();
	xEXTILineConfig();	//DIR(PC11)入力で割り込みがかかるようにする。
	xSetTim2CountDir();

	gT.const_encoder_pulse=1000;	//エンコーダパルス数逓倍前[pulse]
	gT.const_encoder_pulse2=32768;	//基準エンコーダパルス数逓倍前[pulse]

	//gT.const_current_ierr_max=20.0;
	//gT.const_current_pgain=1.0;	//10とかにすると発振してFETが壊れます
	//gT.const_current_igain=0.0;


	//	gT.const_velocity_igain=50.0;
	//	gT.const_velocity_pgain=0.1;
	//	gT.const_position_pgain=100;	//500でFETが壊れます

	//ダイレクト位置制御で概ね良さそうな値
	//	gT.const_position_pgain=100;
	//	gT.const_position_dgain=-0.1;

	//静かに回すのに良さそうな値
	gT.const_position_pgain=0.5;
	gT.const_position_dgain=-0.05;
	//	gT.ref_position_control_velocity=500;


	xTim3Init();	//タイマ割り込み(pd制御)

	int rxnum=0;
//	int pwm=0;
//	int16_t enc;
	float d;

	xUSART2_puts("\r\n\r\n\r\n\r\nwelcome to nucleo F401\r\n");
	xCurrentSensorCalib();
	char buf[259];

	gPulse2Rad=1.0/kEncPulsex4 *2.0*kPI;
	gVolt2Duty=1.0/24*1000;
	float k=0.0;
	float speed=0.0;

	//xSetPwm(-100);

	//TDKセンサのオフセット
	gT.const_magenc_offset=0.0;
	xTdkAllRead();
	//基準エンコーダの初期値をTDKに合わせる
	gT.present_position_pulse2=gT.present_magenc_pos/(2.0*kPI)*gT.const_encoder_pulse2*4;


	while (1)
	{

		int i;
		int dispcount=0;


		for(i=-1000;i<1000;i+=50){
			rxnum=xUSART2_rxnum();
			//ad=ADC_GetConversionValue( ADC1 );


//			enc = TIM_GetCounter(TIM5);         // エンコーダ読み取り


			//get_tick_count(&timestamp);


			/*sprintf(buf,"test. rxnum=%d pwm=%d enc=%x ad0=%x cur=%6.2f tim=%d enc=%d pos=%d\r\n",
					rxnum,pwm,enc,g_ad_value[0],xCurrentRead(),gTimCount,
					enc,gT.present_position_pulse);
			xSetPwm(pwm);
			 */
			/*sprintf(buf,"tim,%d, cur,%6.2f , cur_goal,%6.2f , volt_goal, %6.2f, pgain, %6.2f, igain, %6.2f , ierr,%6.2f,duty,%d\r\n",
							gTimCount,gT.present_current,gT.ref_current_goal
							,gT.ref_voltage_goal,gT.const_current_pgain,gT.const_current_igain
							,gT.present_current_ierr,gT.ref_duty_goal);
			 */
			/*

			sprintf(buf,"tim,%d, vel,%f , vel_goal,%6.2f , cur_goal, %6.2f, pgain, %6.3f, igain, %6.2f ,perr,%6.2f, ierr,%6.2f\r\n",
							gTimCount,
							gT.present_velocity,
							gT.ref_velocity_goal,
							gT.ref_current_goal,
							gT.const_velocity_pgain ,
							gT.const_velocity_igain,
							gT.present_velocity_perr,
							gT.present_velocity_ierr);
			 */

			/*			sprintf(buf,"tim,%d,pos,%f, vel, %f, pos_goal,%f , perr,%f, vel,%6.2f , pos_pgain, %6.3f, vel_by_poscon, %f ,vel_by_poscon_lim,%f\r\n",
							gTimCount,
							gT.present_position,
							gT.present_velocity,
							gT.ref_position_goal,
							gT.present_position_perr,
							gT.ref_position_control_velocity,
							gT.const_position_pgain ,
							gT.present_velocity_goal_by_poscon,
							gT.present_velocity_goal_by_poscon_lim);
			 */
			sprintf(buf,"tim,%ld,pos,%f, vel, %f, pos_goal,%f , perr,%f, derr,%f ,pgain,%f, dgain,%f\r\n",
					gTimCount,
					gT.present_position,
					gT.present_velocity,
					gT.ref_position_goal,
					gT.present_position_perr,
					gT.present_position_derr,
					gT.const_position_pgain,
					gT.const_position_dgain);

			d=(gT.present_magenc_pos-gT.present_position2);
			if(d<(kPI))d+=2.0*kPI;
			if(d>(kPI))d-=2.0*kPI;
			sprintf(buf,"tim,%ld,magraw,%d,pos[deg],%f,tdk[deg],%f, diff[deg],%f\r\n",
					gTimCount,
					gT.present_magenc_raw,
					gT.present_position2/(2.0*kPI)*360,
					gT.present_magenc_pos/(2.0*kPI)*360,
					d*360.0/(2.0*kPI));

			/*			sprintf(buf,"tim,%d,magraw,%d,magturn,%f,posturn,%f,led,%02x,memc,%02x,sts8,%02x,offset,%04x,tim2,%d,int,%d\r\n",
							gTimCount,
							gT.present_magenc_raw,
							(float)gT.present_magenc_raw/65536.0,
							(float)gT.present_position_pulse2/(-32768.0*4),
							gT.present_magenc_led,
							gT.present_magenc_mem_control,
							gT.present_magenc_sts8,
							gT.present_magenc_angoffset,
							(int32_t)TIM_GetCounter(TIM2),
							gLineInt
							);
			 */
			/*sprintf(buf,"tim,%d,magenc_raw=%d,tim4,%d\r\n",
					gTimCount,
					gT.present_magenc_raw,
					TIM_GetCounter(TIM4));
			 */
			if(dispcount%1000==0){
				//				xTdkAllRead();
				//				xUSART2_puts(buf);
			}
			dispcount++;

			//シリアル入力の処理
			if(rxnum>0){
				char c=xUSART2_getc();
				if(c=='d')xUSART2_puts(buf);
				/*
				if(c=='0')pwm=0;
				if(c=='q')pwm+=10;
				if(c=='a')pwm-=10;
				 */
				/*
				if(c=='0')gT.ref_current_goal=0;
				if(c=='1')gT.ref_current_goal+=0.1;
				if(c=='2')gT.ref_current_goal-=0.1;

				if(c=='4')gT.const_current_pgain+=0.1;
				if(c=='5')gT.const_current_pgain-=0.1;
				if(c=='6')gT.const_current_igain+=0.1;
				if(c=='7')gT.const_current_igain-=0.1;
				 */
				/*
				if(c=='0')gT.ref_velocity_goal=0;
				if(c=='q')gT.ref_velocity_goal+=0.1;
				if(c=='a')gT.ref_velocity_goal-=0.1;

				if(c=='w')gT.const_velocity_pgain+=0.001;
				if(c=='s')gT.const_velocity_pgain-=0.001;
				if(c=='e')gT.const_velocity_igain+=0.1;
				if(c=='d')gT.const_velocity_igain-=0.1;
				 */
				//				if(c=='0')gT.const_position_pgain=0;
				//				if(c=='q')gT.const_position_pgain+=1;
				//				if(c=='a')gT.const_position_pgain-=1;

				//				if(c=='w')gT.const_position_dgain+=0.01;
				//				if(c=='s')gT.const_position_dgain-=0.01;
				//				if(c=='e')gT.ref_position_goal+=1;
				//				if(c=='d')gT.ref_position_goal-=1;
				//				if(c=='r')gT.ref_position_goal+=0.01;
				//				if(c=='f')gT.ref_position_goal-=0.01;
				if(c=='0')speed=0.0;
				if(c=='1')speed=0.1;
				if(c=='2')speed=1;
				if(c=='3')speed=3;
				if(c=='4')speed=10;
				if(c=='z')xTdkSetZero();
				if(c=='x')xTdkDebugMode();
				if(c=='o'){
					//オフセット設定(現在の角度をオフセットとして入れる)
					xUSART2_puts("\r\n**Offset write start.\r\n");
					xTdkAllRead();	//すべてのレジスタを読み取る
					xTdkDebugMode();	//デバッグモードに入る
					//オフセット値を書き込み
					xUSART2_puts("\r\nwrite ANG_OFFSET\r\n");
					xTdkWrite16bit(0x148,~gT.present_magenc_raw);
					//オフセット値を読んで、かけているかどうか確認する
					xUSART2_puts("\r\nread ANG_OFFSET\r\n");
					xTdkRead16bit(0x148);

					//NVMへの書き込み
					xTdkDebugMode();
					xUSART2_puts("\r\nStop DSP. write 0 to CNTL\r\n");
					xTdkRead8bit(0x00);	//Stop DSP
					xTdkWrite8bit(0x00,0);	//Stop DSP
					xTdkRead8bit(0x00);	//Stop DSP
					xUSART2_puts("\r\nNVM program. write 1 to MEM_CONTROL2[0]\r\n");
					xTdkWrite8bit(0x2d,1);	//NVM program
					xUSART2_puts("\r\nread MEM_CONTROL\r\n");
					xTdkRead8bit(0x2e);	//mem_control
					xUSART2_puts("\r\nread STS8\r\n");
					xTdkRead8bit(0x1b);	//sts8

				}
				if(c=='p'){
					//組み付け誤差の補償
					xUSART2_puts("\r\n**Assembly inaccuracy compensation.\r\n");
					xTdkDebugMode();	//デバッグモードに入る
					//オフセット値を書き込み
					xUSART2_puts("\r\nwrite 1 to PRE_CALIB[0]\r\n");
					xTdkRead8bit(0x41);
					xTdkWrite8bit(0x41,0x01);
					xTdkRead8bit(0x41);

					/*
					//NVMへの書き込み
					xTdkDebugMode();
					xUSART2_puts("\r\nNVM program. write 1 to MEM_CONTROL2[0]\r\n");
					xTdkWrite8bit(0x2d,1);	//NVM program
					xUSART2_puts("\r\nread MEM_CONTROL\r\n");
					xTdkRead8bit(0x2e);	//mem_control
					xUSART2_puts("\r\nread STS8\r\n");
					xTdkRead8bit(0x1b);	//sts8
					*/
				}
				if(c=='l'){
					//LEDフィールドの表示
					xTdkRead8bit(0x10);
				}
				if(c=='t'){
					xUSART2_puts("\r\nwrite test\r\n");
					xTdkWrite16bit(0x102,0x1234);
					xTdkRead16bit(0x102);
				}
				if(c=='r'){
					xUSART2_puts("\r\bread test\r\n");
					xTdkAllRead();
				}

			}
			//if(c=='p')xTdkWrite16bit(0x41,1);
		}

		Delay(0xFFF);
		//gT.ref_position_goal=sin(k)*2*kPI*4*5/360;
		//gT.ref_position_goal=sin(k)*2*kPI*4;
					gT.ref_position_goal=k;
		k+=speed;





		/*

			SPI3_CS_LOW();
			for(i=0;i<6;i++){
				SPI_I2S_SendData(SPI3, 0x55);
				//SPI読み取り
				while(!SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_IT_TXE));	//受信完了を待つ
				int spi_read = SPI_I2S_ReceiveData(SPI3);
			}
			SPI3_CS_HIGH();
		 */

	}

}

void xTdkDebugMode(void)
{
	//デバッグモードに入る
	xUSART2_puts("\r\nxTdkDebugMode().\r\n");
	xUSART2_puts("\r\nwrite pin code.\r\n");
	xTdkWrite8bit(0x3fc,0x29);
	xTdkWrite8bit(0x3fd,0x39);
	xTdkWrite8bit(0x3fe,0x6d);
	xTdkWrite8bit(0x3ff,0x23);
//	xUSART2_puts("\r\nNVM program. write 1 to MEM_CONTROL2[0]\r\n");
//	xTdkWrite8bit(0x2d,1);	//NVM program
//	xUSART2_puts("\r\nread MEM_CONTROL\r\n");
//	xTdkRead8bit(0x2e);	//mem_control
//	xUSART2_puts("\r\nread STS8\r\n");
//	xTdkRead8bit(0x1b);	//sts8
}
void xTdkSetZero(void)
{
	//TDKセンサの現在の位置を0とする

}
void xTdkAllRead(void)
{
	//tdkセンサから現在角度とステータスを読んでコントロールテーブルに入れる
	//TDKセンサの角度レジスタを読み込む
	//	gT.present_magenc_raw=(xTdkRead8bit(0x06)<<8)|xTdkRead8bit(0x07);
	xUSART2_puts("xTdkAllRead()\r\n");
	gT.present_magenc_raw=xTdkRead16bit(0x06);
	gT.present_magenc_pos=((float)gT.present_magenc_raw)/65536.0*2.0*kPI - gT.const_magenc_offset;
	gT.present_magenc_angoffset=xTdkRead16bit(0x148);
	//TDKセンサのLEDレジスタの値を読む
	gT.present_magenc_led=xTdkRead8bit(0x10);
	gT.present_magenc_mem_control=xTdkRead8bit(0x2d);
	gT.present_magenc_sts8=xTdkRead8bit(0x1b);
	xUSART2_puts("xTdkAllRead() done.\r\n");

}
uint8_t xTdkRead8bit(uint16_t adr)
{
	//tdkセンサのスタートアドレスから1バイトを読んで返す
	int i;
	unsigned char readdata[10];
	char buf[256];
	unsigned char tdk_spi[kSpiReadMax];
	tdk_spi[0]=0x40|((adr>>8)&0x0f);	//read command 8bit access | アドレス上位
	tdk_spi[1]=adr&0xff;	//アドレス下位
	tdk_spi[2]=crc8(tdk_spi,2);
	tdk_spi[3]=0xff;
	tdk_spi[4]=0xff;
	tdk_spi[5]=0xff;


	sprintf(buf,"xTdkRead8bit(adr=%04x) \r\n",adr);
	xUSART2_puts(buf);
	xUSART2_puts("send:");
	for(i=0;i<6;i++){
		sprintf(buf,"%02x, ",tdk_spi[i]);
		xUSART2_puts(buf);
	}


	SPI2_CS_LOW();
	for(i=0;i<6;i++){
		SPI_I2S_SendData(SPI2, tdk_spi[i]);
		//SPI読み取り
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_IT_TXE));	//受信完了を待つ
		readdata[i] = SPI_I2S_ReceiveData(SPI2);
	}
	SPI2_CS_HIGH();


	xUSART2_puts("\r\nrecv:");
	for(i=0;i<6;i++){
		sprintf(buf,"%02x, ",readdata[i]);xUSART2_puts(buf);
	}
	xUSART2_puts("\r\n");


	sprintf(buf,"xTdkRead8bit done. adr=%04x, value=%02x\r\n\r\n",adr, readdata[3]);
	xUSART2_puts(buf);

	return readdata[3];
}
uint16_t xTdkRead16bit(uint16_t adr)
{
	//tdkセンサのスタートアドレスから1バイトを読んで返す
	int i;
	unsigned char readdata[10];
	char buf[256];
	unsigned char tdk_spi[kSpiReadMax];
	tdk_spi[0]=0x50|((adr>>8)&0x0f);	//read command 16bit access | アドレス上位
	tdk_spi[1]=adr&0xff;	//アドレス下位
	tdk_spi[2]=crc8(tdk_spi,2);
	tdk_spi[3]=0xff;
	tdk_spi[4]=0xff;
	tdk_spi[5]=0xff;
	tdk_spi[6]=0xff;


	sprintf(buf,"xTdkRead16bit(adr=%04x) \r\n",adr);
	xUSART2_puts(buf);
	xUSART2_puts("send:");
	for(i=0;i<7;i++){
		sprintf(buf,"%02x, ",tdk_spi[i]);
		xUSART2_puts(buf);
	}


	SPI2_CS_LOW();
	for(i=0;i<7;i++){
		SPI_I2S_SendData(SPI2, tdk_spi[i]);
		//SPI読み取り
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_IT_TXE));	//受信完了を待つ
		readdata[i] = SPI_I2S_ReceiveData(SPI2);
	}
	SPI2_CS_HIGH();


	xUSART2_puts("\r\nrecv:");
	for(i=0;i<7;i++){
		sprintf(buf,"%02x, ",readdata[i]);xUSART2_puts(buf);
	}
	xUSART2_puts("\r\n");

	sprintf(buf,"xTdkRead16bit done. adr=%04x, value=%04x\r\n\r\n\r\n",adr, (((int16_t)readdata[3])<<8)|((int16_t)readdata[4]&0xff));
	xUSART2_puts(buf);



	return (((int16_t)readdata[3])<<8)|((int16_t)readdata[4]&0xff);
}
void xTdkWrite8bit(uint16_t adr, uint8_t writedata)
{
	//tdkセンサのadrにwritedatを書き込む
	int i;
	char buf[256];
	unsigned char readdata[10];
	unsigned char tdk_spi[kSpiReadMax];
	tdk_spi[0]=0x80|((adr>>8)&0x0f);;	//write command 8bit access
	tdk_spi[1]=adr&0xff;
	tdk_spi[2]=writedata;
	tdk_spi[3]=crc8(tdk_spi,3);
	tdk_spi[4]=0xff;
	tdk_spi[5]=0xff;




	sprintf(buf,"xTdkWrite8bit(adr=%04x, data=%02x) \r\n",adr,writedata);
	xUSART2_puts(buf);
	xUSART2_puts("send:");
	for(i=0;i<6;i++){
		sprintf(buf,"%02x, ",tdk_spi[i]);
		xUSART2_puts(buf);
	}
	xUSART2_puts("\r\nrecv:");


	SPI2_CS_LOW();
	for(i=0;i<6;i++){
		SPI_I2S_SendData(SPI2, tdk_spi[i]);
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_IT_TXE));	//受信完了を待つ
		readdata[i] = SPI_I2S_ReceiveData(SPI2);
	}
	SPI2_CS_HIGH();


	for(i=0;i<6;i++){
		sprintf(buf,"%02x, ",readdata[i]);xUSART2_puts(buf);
	}
	xUSART2_puts("\r\n");

}

void xTdkWrite16bit(uint16_t adr, uint16_t writedata)
{
	//tdkセンサのadrにwritedatを書き込む
	int i;
	char buf[256];
	unsigned char readdata[10];
	unsigned char tdk_spi[kSpiReadMax];
	tdk_spi[0]=0x90|((adr>>8)&0x0f);;	//write command 8bit access
	tdk_spi[1]=adr&0xff;
	tdk_spi[2]=(writedata>>8)&0xff;
	tdk_spi[3]=writedata&0xff;
	tdk_spi[4]=crc8(tdk_spi,4);
	tdk_spi[5]=0xff;
	tdk_spi[6]=0xff;


	sprintf(buf,"xTdkWrite16bit(adr=%04x, data=%02x) \r\n",adr,writedata);xUSART2_puts(buf);
	xUSART2_puts("send:");

	for(i=0;i<7;i++){
		sprintf(buf,"%02x, ",tdk_spi[i]);
		xUSART2_puts(buf);
	}
	xUSART2_puts("\r\nrecv:");

	SPI2_CS_LOW();

	for(i=0;i<7;i++){
		SPI_I2S_SendData(SPI2, tdk_spi[i]);
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_IT_TXE));	//受信完了を待つ
		readdata[i] = SPI_I2S_ReceiveData(SPI2);
	}
	SPI2_CS_HIGH();

	for(i=0;i<7;i++){
		sprintf(buf,"%02x, ",readdata[i]);xUSART2_puts(buf);
	}
	xUSART2_puts("\r\n");


}
void xLed(int sw)
{
	if(sw){
		//LED点灯
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
}

void xCurrentSensorCalib()
{
	//電流センサの平均値をとり、
	float sum=0;
	int i;
	char buf[256];
	xUSART2_puts("current sensor calib.\r\n");
	for(i=0;i<100;i++){
		sum+=g_ad_value[0];
//		sprintf(buf,"%d,adval=%d\r\n",i,g_ad_value[0]);
//		xUSART2_puts(buf);
		Delay(0x10000);
	}
	gT.present_current_sensor_offset=(sum/100.0);
	sprintf(buf,"present_current_sensor_offset=%d\r\n",gT.present_current_sensor_offset);
	xUSART2_puts(buf);
}


void xMotorControl()
{

}



float xCurrentRead(void)
{
	//電流値[A]を取得
	float raw=(float)((g_ad_value[0])-gT.present_current_sensor_offset);
	//170ns
	//float current=raw/4096.0*3.3/0.185;	//ACS712出力は185mV/A、AD分解能12ビット=4096、ADレンジ=3.3Vより //9.4us
	float current=raw*0.004354941;	//1.52us
	return current;
}

//CRC8の算出

#define MSB_CRC8    (0x1d)
static unsigned char crc8( const void *buff, size_t size )
{
	unsigned char *p = (unsigned char *)buff;
	unsigned char crc8;

	for ( crc8 = 0xff ; size != 0 ; size-- ){
		crc8 ^= *p++;
		int i;
		for (i = 0 ; i < CHAR_BIT ; i++ ){
			if ( crc8 & 0x80 ){
				crc8 <<= 1;
				crc8 ^= MSB_CRC8;
			}
			else{
				crc8 <<= 1;
			}
		}
	}
	return crc8;
}


//PC11を入力にして外部割込みがかかるようにする
void xEXTILineConfig(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	// Enable GPIOC clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// PC11を入力ピンに設定
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PC11をEXTIにつなぐ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);

	// EXTIの15_10ラインにPC11をつないで、いずれかの変化で割り込みがかかるようにしたい
	// Configure EXTI Line11
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI15_10 Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// GPIO割り込み PC11の変化で割り込みがかかる
void EXTI15_10_IRQHandler(void)
{

	EXTI_ClearITPendingBit(EXTI_Line11);   // 割り込みペンディングビットをクリア
	xSetTim2CountDir();
	gLineInt++;
}
void xSetTim2CountDir(void)
{
	//PC11(DIR入力)に従ってTIM2のカウント方向を変える
	int pc11;
	pc11=(GPIO_ReadInputData(GPIOC)>>11)&0x01;	//PC11を読み取ってTIM2のカウント方向を変化する
	if(pc11==1){
		TIM2->CR1=(TIM2->CR1 | 0x0010);	//downcount;
	}else{
		TIM2->CR1=(TIM2->CR1 & 0xffef);	//upcount
	}

}
/// @brief タイマ3割り込みハンドラ
/// @note 割り込み周期10KHz
void TIM3_IRQHandler (void)
{
	xLed(1);
	// TIM2_FREQUENCY周期での処理
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit (TIM3, TIM_IT_CC1);
	}

	//計測=============================================================
	//モータ軸位置[pulse]
	int16_t enc = TIM_GetCounter(TIM5);
	int16_t diff=enc-g_motor_pos_pre;
	g_motor_pos_pre=enc;
	gT.present_position_pulse+=diff;
	gT.present_position = ((float)gT.present_position_pulse) / (gT.const_encoder_pulse*4.0) *2.0*kPI;	//現在位置

	//基準エンコーダ位置[pulse]
	int16_t enc2 = TIM_GetCounter(TIM4);
	int16_t diff2=enc2-g_motor_pos_pre2;
	g_motor_pos_pre2=enc2;
	gT.present_position_pulse2-=diff2;
	gT.present_position2 = ((float)gT.present_position_pulse2) / (gT.const_encoder_pulse2*4.0) *2.0*kPI;	//現在位置

	//現在電流値
	gT.present_current=xCurrentRead();
	//モーター速度[rad/s]
	gT.present_position_pulse_diff = gT.present_position_pulse-gT.present_position_pulse_pre;
	gT.present_position_pulse_pre = gT.present_position_pulse;
	gT.present_pulse_velocity = gT.present_position_pulse_diff * kSamplingFreq;	//回転速度[pulse/s]
	gT.present_velocity = xLpf(gT.present_pulse_velocity *gPulse2Rad,0);	//回転速度[rad/s]
	//  gT.present_velocity = gT.present_pulse_velocity *gPulse2Rad;	//回転速度[rad/s]
	//モータ位置[rad]


	//制御=============================================================
	//位置制御の計算(直接電圧出力を得る)
	gT.present_position_perr = gT.ref_position_goal - gT.present_position;
	gT.present_position_derr = gT.present_velocity;
	gT.ref_voltage_goal=gT.present_position_perr*gT.const_position_pgain + gT.present_position_derr*gT.const_position_dgain;
	gT.ref_duty_goal = gT.ref_voltage_goal*gVolt2Duty;	//voltageからdutyへの変換
	xSetPwm(gT.ref_duty_goal);





	//位置制御の計算(速度ループを介する)
	/*
  gT.present_position_perr = gT.ref_position_goal - gT.present_position;
  gT.present_velocity_goal_by_poscon = gT.present_position_perr * gT.const_position_pgain;
  float vel=gT.present_velocity_goal_by_poscon;
  if(vel > fabs(gT.ref_position_control_velocity))vel=fabs(gT.ref_position_control_velocity);
  if(vel < (-fabs(gT.ref_position_control_velocity)))vel=-fabs(gT.ref_position_control_velocity);
  gT.ref_velocity_goal=vel;

  //速度制御の計算
  gT.present_velocity_perr = gT.ref_velocity_goal - gT.present_velocity;	//速度比例誤差の計算
  gT.present_velocity_ierr += gT.present_velocity_perr*kSamplingTime;	//速度積分誤差の計算
  gT.ref_current_goal = -(gT.present_velocity_perr * gT.const_velocity_pgain + gT.present_velocity_ierr*gT.const_velocity_igain);
	 */
	//電流制御の計算
	//gT.present_current_perr = gT.ref_current_goal-gT.present_current;	//電流比例誤差の計算
	//gT.present_current_ierr += gT.present_current_perr*kSamplingTime;	//電流積分誤差の計算

	//ierr_maxをクランプ
	//if(gT.present_current_ierr>gT.const_current_ierr_max)gT.present_current_ierr=gT.const_current_ierr_max;
	//if(gT.present_current_ierr<(-gT.const_current_ierr_max))gT.present_current_ierr=(-gT.const_current_ierr_max);
	//gT.ref_voltage_goal = gT.present_current_perr*gT.const_current_pgain + gT.present_current_ierr*gT.const_current_igain;	//電圧指令値の計算

	//電流制御ループを回すとFETが壊れやすいので、電流制御しないことにする。
	//gT.ref_voltage_goal = gT.ref_current_goal*1.0;
	//  gT.ref_duty_goal = gT.ref_voltage_goal*gVolt2Duty;	//voltageからdutyへの変換


	//電圧値を出力
	//  xSetPwm(gT.ref_duty_goal);
	//xSetPwm(10);


	gTimCount++;
	xLed(0);
}

//===============================================================================
void xTim2Init(void)
{
	//TIM2は外部入力をカウントする

	//PA5をTIM2_ETRにする
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // GPIOのピンを入出力以外に使う設定。超重要。
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	// GPIO PA5のAFをTIM2_ETRにする
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);  // TIM2 ETR入力

	TIM_TimeBaseInitTypeDef  TIM2_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM2_OCInitStructure;
	TIM_DeInit (TIM2);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);	//tim2クロックは84MHz。
	TIM2_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM2_TimeBaseStructure.TIM_Prescaler = 0;	//プリスケーラ。0だと1分周。
	TIM2_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;;	//外部クロックサンプル
	TIM2_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit (TIM2, &TIM2_TimeBaseStructure);

	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = 0;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init (TIM2, &TIM2_OCInitStructure);
	TIM_OC1PreloadConfig (TIM2, TIM_OCPreload_Enable);
	TIM_ITConfig (TIM2, TIM_IT_CC1, ENABLE);

	//外部トリガの設定。プリスケーラなし、極性を反転しない、フィルタ値0(0x00-0x0fの間で設定する)
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);

	TIM_Cmd (TIM2, ENABLE);
}

void xTim3Init(void)
{
	//TIM3はタイマ割り込み(10KHz)で制御ループを回す
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            /* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  /* value = 0 - 15*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         /* value = 0 - 15*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



	TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM3_OCInitStructure;
	TIM_DeInit (TIM3);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);	//tim2クロックは84MHz。
	TIM3_TimeBaseStructure.TIM_Period = (84000000 / 10000) - 1;	//=8400　これで10KHz
	TIM3_TimeBaseStructure.TIM_Prescaler = 0;	//プリスケーラ。0だと1分周。
	TIM3_TimeBaseStructure.TIM_ClockDivision = 0;	//外部クロックサンプルの間隔。使用しないので0。
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit (TIM3, &TIM3_TimeBaseStructure);

	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 0;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init (TIM3, &TIM3_OCInitStructure);
	TIM_OC1PreloadConfig (TIM3, TIM_OCPreload_Enable);
	TIM_ITConfig (TIM3, TIM_IT_CC1, ENABLE);
	TIM_Cmd (TIM3, ENABLE);
}


void xADCInit()
{
	/// @brief AD変換初期化
	/// @note チャンネル設定のみハンド仕様
	/// @note 元のxADCInit
	// A/D変換が終了すると値はDMAにより変数g_ad_valueに転送される
	//PC14 電流センサ
	//PC0 予備


	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);


	//GPIOのピンをADC入力に設定する
	//GPIO_Cの出力ポート設定
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// DMA2 Stream0 channel0 configU3ration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&g_ad_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;	//ADCチャンネル数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init (DMA2_Stream0, &DMA_InitStructure);

	DMA_Cmd (DMA2_Stream0, ENABLE);

	// ADC Common configU3ration
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInit (&ADC_CommonInitStructure);

	// ADC1 regU3lar channel 12 configU3ration
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;	//ADCの本数
	ADC_Init (ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_480Cycles);
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  2, ADC_SampleTime_480Cycles);


	// Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);
	// Enable DMA request after last transfer
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	// Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADC1);


}

void xGPIOInit()
{
	/* GPIOD Periph clock enable */
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//GPIO_Aの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_Bの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_Cの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_Dの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


}


void Delay(__IO uint32_t nCount)
{
	while(nCount--)
	{
	}
}

// TB6643むけPWM出力
// IN1=PA9(TIM1_CH2)
// IN1=PA10(TIM1_CH3)
// dは(-1000)～1000 0が停止
void xSetPwm(int16_t duty)
{

	if(duty<-kPWMMAX)duty=-kPWMMAX;
	if(duty>kPWMMAX)duty=kPWMMAX;


	if(duty >= 0) {
		// 正転 U=PWM_HiZ(PB15_CH3N=Lo CH3_PA10=PWM) V=GND(PB14_CH2N=Hi PA9_CH2=Lo)
		// PA10=CH3 =PMW+
		// PA9_ CH2 =L

		//                                                                             1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xffcfffff) | 0x00200000;   // PA10 = AF[10] &1111 1111 1100 1111 1111 1111 1111 1111 | 0000 0000 0010 0000 0000 0000 0000 0000
		GPIOA->AFR[10] = 0x1;    // TIM1

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xfff3ffff) | 0x00040000;   // PA9 = GPIO[01] &1111 1111 1111 0011 1111 1111 1111 1111 | 0000 0000 0000 0100 0000 0000 0000 0000
		//    GPIOA->BSRRL = _PA9;    // PA9=H (BSRRLに書くとH BSRRHに書くとLになる PWMオフ時道通
		GPIOA->BSRRH = _PA9;    // PA9=H (BSRRLに書くとH BSRRHに書くとLになる PWMオフ時HiZ


		TIM1->CCR3 = duty;
	}
	else {
		// 逆転 U=GND(PB15_CH3N=Hi CH3_PA10=Lo) V=PWM-HiZ(PB14_CH2N=Lo PA9_CH2=PWM)
		// PA10=CH3 =L
		// PA9_ CH2 =PWM+

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xffcfffff) | 0x00100000;   // PA10 = GPIO[01] &1111 1111 1100 1111 1111 1111 1111 1111 | 0000 0000 0001 0000 0000 0000 0000 0000
		//   GPIOA->BSRRL = _PA10;    //PA10=H (BSRRLに書くとH BSRRHに書くとLになる
		GPIOA->BSRRH = _PA10;    //PA10=H (BSRRLに書くとH BSRRHに書くとLになる

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xfff3ffff) | 0x00080000;   // PA9 = AF[10] &1111 1111 1111 0011 1111 1111 1111 1111 | 0000 0000 0000 1000 0000 0000 0000 0000
		GPIOA->AFR[9] = 0x1;   // AF=TIM1


		//pwm = -pwm;
		TIM1->CCR2 = -duty;   //kPWMMAXをCCR2に入れた時、100%dutyになる
	}
}


/// @brief TIM1（PWM）の初期化
/// @note アームと処理は変わらない
void xTim1Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
	uint16_t TimerPeriod1 = 0;

	TIM_DeInit (TIM1);
	// TimerPeriod1=PWM周期[tick]。
	// フリーランカウンタの上限値を決める。TIM_FREQUENCYはPWM周波数を表す。50KHzなので50000。
	// すなわちTimerPeriod1=2400[tick]。1tick=1/120MHz[sec]
	// 2400[tick]=2400*(1/120M)=20[us]
	// 1/20[us]=50[KHz]となる。
	//  TimerPeriod1 = (4200) - 1;		//10KHz. 84000000/20000=4200でフルスケール。
	TimerPeriod1 = (kPWMMAX) - 1;		//16.8MHz/1000=16.8KHz. 1000でフルスケール。

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM_TimeBaseStructure.TIM_Prescaler = 5;	//84M/5=16.8MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // TIMx_CNT < TIMx_CCRyであるときOCがHになるモード。つまりCCRレジスタが0で停止、TimerPeriodでMAX出力。

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 補相出力（反転したもの）の出力を有効化

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期のCCRレジスタの値を決める。
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期の出力
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期の出力
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

	TIM1_BDTRInitStructure.TIM_DeadTime = 48;  // 0.4usのデッドバンド。単位は[tick] 1tick=1/120MHz

	TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

	TIM_Cmd(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	//GPIOの出力をTIM1に切り替える
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);


}


void xSpi2Init(void)
{
	//SPI2信号をGPIOに割り当てる
	//PB13 SPI1_SCK
	//PB14 SPI1_MISO
	//PB15 SPI1_MOSI
	//PB6 SPI1_CS	これはGPIOの出力ポート


	//SPIモジュールの初期化
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB1PeriphClockCmd (RCC_APB1Periph_SPI2, ENABLE);

	// Chip Select high
	SPI2_CS_LOW();

	// SPI configuration
	//**** Function as the limit up to 15 bits
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//8bit単位の送信/受信
	//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;	//16bit単位の送信/受信
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	//クロック極性　クロックなしの時Lo
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//clk=2.5MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	//	SPI_InitStructure.SPI_CRCPolynomial = 8;

	SPI_Init (SPI2, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd (SPI2, ENABLE);


	//GPIOのポートをSPIに割り当てる
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);	//MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);	//MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);	//SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);	//NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void xSpi3Init(void)
{
	//SPI3信号をGPIOに割り当てる
	//PC10 SPI3_SCK
	//PC11 SPI3_MISO
	//PC12 SPI3_MOSI
	//PD2  SPI3_CS	これはGPIOの出力ポート


	//SPIモジュールの初期化
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB1PeriphClockCmd (RCC_APB1Periph_SPI3, ENABLE);

	// Chip Select high
	SPI3_CS_HIGH();

	// SPI configuration
	//**** Function as the limit up to 15 bits
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//8bit単位の送信/受信
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//clk=2.5MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	//	SPI_InitStructure.SPI_CRCPolynomial = 8;

	SPI_Init (SPI3, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd (SPI3, ENABLE);


	//GPIOのポートをSPIに割り当てる
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);	//MOSI
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);	//MISO
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);	//SCK
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
// エンコーダカウンタの初期化
// TIM5をエンコーダカウンタとして使う
// 入力ピン PA0=TIM5_CH1 PA1=TIM5_CH2
void xTim5Init(void)
{
	TIM_TimeBaseInitTypeDef ts;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // GPIOのピンを入出力以外に使う設定。超重要。
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO PA0とPA1をTIM5につなぐ
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);  // TIM5 A相入力
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);  // TIM5 B相入力

	// TIM5をエンコーダカウンタとして設定
	TIM_DeInit(TIM5);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	ts.TIM_Period = 0x0ffff;
	ts.TIM_Prescaler = 0;
	ts.TIM_ClockDivision = TIM_CKD_DIV1;
	ts.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_EncoderInterfaceConfig(TIM5,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    // 4逓倍
	TIM_TimeBaseInit(TIM5, &ts);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInit(TIM5,&TIM_ICInitStructure);

	TIM_Cmd(TIM5, ENABLE);

	TIM_SetCounter(TIM5, 0);
}

// Tim4エンコーダカウンタの初期化
// TIM4をエンコーダカウンタとして使う
// 入力ピン PB6=TIM4_CH1=A相 PB7=TIM4_CH2=B相
void xTim4Init(void)
{

	// GPIO B6,B7をエンコーダパルス入力として使う
	TIM_TimeBaseInitTypeDef ts;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // GPIOのピンを入出力以外に使う設定。超重要。
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// GPIO PB6とPB7をTIM4につなぐ
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);  // TIM4 A相入力
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);  // TIM4 B相入力

	// TIM4をエンコーダカウンタとして設定
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	ts.TIM_Period = 0x0ffff;
	ts.TIM_Prescaler = 0;
	ts.TIM_ClockDivision = TIM_CKD_DIV1;
	ts.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);    // 4逓倍
	TIM_TimeBaseInit(TIM4, &ts);

	//入力フィルタの設定
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_SetCounter(TIM4,0);

	TIM_Cmd(TIM4, ENABLE);


}

float xLpf(float in, int reset)
{
	//IIRフィルタ
	//reset=1の時リセットする
	float t;
	static float d1, d2;
	//   static float k = 0.125; /* 0 < k < 1 */
	static float k = 0.5; /* 0 < k < 1 */

	if (!reset) {
		t = d1;
		d1 += (d2 - in) * k;
		d2 -= (t + d2 * 2.0) * k; /* Q = 0.5 */
		return d2;
	}else{
		d1 = -2.0 * in;
		d2 = in;
		return d2;
	}
}

