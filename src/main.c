
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


#define kSampleNum (1000)	//サンプル点数

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
float xCurrentRead(void);
static unsigned char  crc8(const void* buff, size_t size);
void xADCInit();
void xLed(int sw);
void xTim3Init(void);

#define SPI2_CS_HIGH()	GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define SPI2_CS_LOW()	GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define SPI3_CS_HIGH()	GPIO_SetBits(GPIOD, GPIO_Pin_2)
#define SPI3_CS_LOW()	GPIO_ResetBits(GPIOD, GPIO_Pin_2)

TControlTable gT;
uint8_t g_ad_value[kSampleNum];
int32_t gTimCount;
uint16_t g_motor_pos_pre=0;
uint16_t g_motor_pos_pre2=0;
float gPulse2Rad;
float gVolt2Duty;
long gLineInt;
int gAdFlag=0;

#define kSpiReadMax 100

int main(void)
{

	xGPIOInit();
	xSpi2Init();
	xADCInit();
	xUSART2_init();


	//xTim3Init();	//タイマ割り込み(pd制御)


	xUSART2_puts("\r\n\r\n\r\n\r\nwelcome to nucleo F401\r\n");

	char buf[259];
	int rxnum;

	while (1)
	{

		int i;
		int dispcount=0;


		rxnum=xUSART2_rxnum();

		dispcount++;

		//シリアル入力の処理
		if(rxnum>0){
			char c=xUSART2_getc();
			if(c=='d')xUSART2_puts(buf);
			if(c=='s'){
				xLed(1);

				gAdFlag=0;
				ADC_DeInit();
				xADCInit();
				xLed(0);

//				DMA_Cmd (DMA2_Stream0, ENABLE);
//				ADC_Cmd(ADC1, ENABLE);
//				ADC_SoftwareStartConv(ADC1);
			}
		}
		if(gAdFlag==1){

			int i;
			xUSART2_puts("start\r\n");
			xLed(0);
			for(i=0;i<kSampleNum;i++){
				sprintf(buf,"%03d\r\n",g_ad_value[i]);
				xUSART2_puts(buf);
			}
			xLed(1);
			xUSART2_puts("end\r\n");
			//再開
			gAdFlag=0;
			ADC_DeInit();
			xADCInit();


		}
		//if(c=='p')xTdkWrite16bit(0x41,1);

		Delay(0xFFF);
//		sprintf(buf,"0,%d,1,%d\r\n",g_ad_value[0],g_ad_value[1]);
//		xUSART2_puts(buf);

	}

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


/// @brief タイマ3割り込みハンドラ
/// @note 割り込み周期10KHz
void TIM3_IRQHandler (void)
{
	xLed(1);
	// TIM2_FREQUENCY周期での処理
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit (TIM3, TIM_IT_CC1);
	}
	xLed(0);
}

//===============================================================================
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
	//PC2 ch12
	//PC3 ch13


	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;


	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);


	//GPIOのピンをADC入力に設定する
	//GPIO_Cの出力ポート設定
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
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
	DMA_InitStructure.DMA_BufferSize = kSampleNum;	//転送回数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//ペリフェラルのデータサイズ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	//メモリのデータサイズ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//ペリフェラルのデータサイズ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//メモリのデータサイズ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//繰り返し実行
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		//1回の転送で終わり
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init (DMA2_Stream0, &DMA_InitStructure);

	//DMAの転送が終わったら割り込みがかかるようにする
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

	DMA_Cmd (DMA2_Stream0, ENABLE);


	ADC_DeInit();	//これがないとADC値が化けることがある。

	// ADC Common configU3ration
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInit (&ADC_CommonInitStructure);

	// ADC1 regU3lar channel 12 configU3ration
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;	//ADCの本数
	ADC_Init (ADC1, &ADC_InitStructure);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_3Cycles);


	// Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);
	//変換結果がDMA転送されるごとに、ADCは次の変換を開始するように設定
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	//
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	// Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADC1);


}


void DMA2_Stream0_IRQHandler(void)
{
	/* Test on DMA Stream Half Transfer interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		/* Clear DMA Stream Half Transfer interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);

		xLed(1);
		xLed(0);
		// Add code here to process first half of buffer (ping)
	}

	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		/* Turn LED3 on: End of Transfer */
		xLed(1);
		xLed(0);
		gAdFlag=1;	//変換終了フラグ

		// Add code here to process second half of buffer (pong)
	}
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
