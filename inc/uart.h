
/**********************************************************************
File    : uart.h
Purpose :
**********************************************************************/
#ifndef __UART_H__
#define __UART_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
/***************************** Prototypes *****************************/
void USART_Config(void);
int putc2 ( int ch );
void puts2(char* s);
void gets2(unsigned char* s);
unsigned char getc2(void);
int rxnum2(void);
int rxlinenum2(void);
#endif // __UART_H__


