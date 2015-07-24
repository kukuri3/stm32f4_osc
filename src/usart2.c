//STM32F401用USART2用送受信
//送信：ポーリング
//受信：割り込み




#include  <stm32f4xx.h>
#include <stdbool.h>	//bool型を使うため
#include <stdint.h>		//NULLを使うため
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "usart2.h"


//////////////////  USART2
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2


//usart2用受信割り込みバッファ
#define EXUSART2_RX_BUFSIZE (1024)
#define EXUSART2_RX_BUFMAX  (1024) // 実際に使用する最大数
unsigned char gUSART2_RxBuf[EXUSART2_RX_BUFSIZE];	//バッファ本体
int gUSART2_RxWPtr;	//書き込みポインタ
int gUSART2_RxRPtr;	//読み込みポインタ
//int gRxNum;		//受信バイト数
int gUSART2_gRxLineNum=0;	//受信行数

//USART2割り込みハンドラ
void USART2_IRQHandler(void)
{

  //とりあえず受信エコーバックテスト
  /*
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
  {
  	unsigned char r;
  	r=USART_ReceiveData(USART2); //受信したデータ読み込み
  	USART_SendData(USART2,r); //そのデータを送信
  }
  */

//	unsigned char rxWrNext;
  if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)	//受信割り込み
  {
    unsigned char c;
    c=USART_ReceiveData(USART2);
    if(c==0x0a)goto ex;	//LFは無視する

    if(xUSART2_rxnum()<EXUSART2_RX_BUFSIZE) {
      //バッファが空いていれば受信して受信バッファに入れる
      if(xUSART2_rxnum()<EXUSART2_RX_BUFMAX) {
        gUSART2_RxBuf[gUSART2_RxWPtr]=c;
        gUSART2_RxWPtr++;
        if(gUSART2_RxWPtr>=EXUSART2_RX_BUFSIZE)gUSART2_RxWPtr=0;
        //gRxNum++;	//受信バイト数をインクリメント
        //行数をカウント
        if(c==0x0d)gUSART2_gRxLineNum++;
      }
    }//バッファがいっぱいだったら捨てる
ex:
    ;

  }

}
void xUSART2_init(void)
{
	  USART_InitTypeDef USART_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* Peripheral Clock Enable -------------------------------------------------*/
	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

	  /* Enable USART clock */
	  USARTx_CLK_INIT(USARTx_CLK, ENABLE);

	  /* USARTx GPIO configuration -----------------------------------------------*/
	  /* Connect USART pins to AF7 */
	  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
	  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

	  /* Configure USART Tx and Rx as alternate function push-pull */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN | USARTx_RX_PIN;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
	  //GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
	  //GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
	  //GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate =  921600;	//115200;460800;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  /* When using Parity the word length must be configured to 9 bits */
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USARTx, &USART_InitStructure);
	  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

	  /* Enable USART */
	  USART_Cmd(USARTx, ENABLE);


  //受信割り込みを発生させるようにする
  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	//受信割り込みを許可

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
  USART2->CR1 |= USART_CR1_UE;                        // USART enable

//	USART_ClearFlag(USART2,USART_FLAG_TC);

  //受信バッファの初期化
  gUSART2_RxWPtr=0;
  gUSART2_RxRPtr=0;
  gUSART2_gRxLineNum=0;

  //USART2受信割り込み開始
//	ena_int(USART2_IRQn+16);
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// this sets the priority group of the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART2 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);			// the properties are passed to the NVIC_Init function which takes care of the low level stuff

}
int xUSART2_rxnum(void)
//受信バイト数を返す
{
  int n;
  n=gUSART2_RxWPtr-gUSART2_RxRPtr;
  if(n<0)n+=EXUSART2_RX_BUFSIZE;
  return n;
}
int xUSART2_rxlinenum(void)
//受信行数を返す
{
  return gUSART2_gRxLineNum;
}

unsigned char xUSART2_getc(void)
//一文字を受信バッファから取り出す
{
  while(xUSART2_rxnum()<=0) {};
  unsigned char c;
  c=gUSART2_RxBuf[gUSART2_RxRPtr];
  gUSART2_RxRPtr++;
  if(gUSART2_RxRPtr>=EXUSART2_RX_BUFSIZE)gUSART2_RxRPtr=0;
  //gRxNum--;
  return c;
}
void xUSART2_gets(unsigned char* s)
//一行を受信バッファから取り出す
{
  if(gUSART2_gRxLineNum<=0) {};
  while(1) {
    unsigned char c;
    c=xUSART2_getc();
    *s=c;
    s++;
    if(c==0x0d) {	//行末に達した
      *s=0;
      gUSART2_gRxLineNum--;
      return;
    }
  }
}
void xUSART2_putc(unsigned char c)
//一文字送信
{
//	usart3_puts("xUSART2_puts()\r\n");
  USART_SendData(USART2, (uint8_t)c);
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
//		usart3_puts("xUSART2_puts() waiting flag change.\r\n");
//		tslp_tsk (100);

  };	//送信終了を待つ
}
void xUSART2_puts(char* s)
//文字列の送信
{
  while(*s!=0) {
    xUSART2_putc((unsigned char)*s);
    s++;
  }
}





