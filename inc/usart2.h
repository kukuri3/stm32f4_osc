#ifndef __USART2_H
#define __USART2_H


void xUSART2_puts(char* s);
void xUSART2_putc(unsigned char c);
void xUSART2_gets(unsigned char* s);
unsigned char xUSART2_getc(void);
int xUSART2_rxnum(void);
void xUSART2_init(void);
int xUSART2_rxlinenum(void);



#endif
