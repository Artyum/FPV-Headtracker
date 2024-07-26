#ifndef _UART_H_
#define _UART_H_

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>


// http://wormfood.net/avrbaudcalc.php

#if F_CPU==8000000

#define UBRR_4800		103
#define UBRR_9600		51
#define UBRR_14400		34
#define UBRR_19200		25
#define UBRR_38400		12
#define UBRR_57600		8
#define UBRR_115200		3
#define UBRR_250000		1

#elif F_CPU==10000000

#define UBRR_4800		129
#define UBRR_9600		64
#define UBRR_14400		42
#define UBRR_19200		32
#define UBRR_38400		15
#define UBRR_57600		10
#define UBRR_115200		4

#elif F_CPU==11059200

#define UBRR_4800		143
#define UBRR_9600		71
#define UBRR_14400		47
#define UBRR_19200		35
#define UBRR_38400		17
#define UBRR_57600		11
#define UBRR_115200		5

#elif F_CPU==12000000

#define UBRR_4800		155
#define UBRR_9600		77
#define UBRR_14400		51
#define UBRR_19200		38
#define UBRR_38400		19
#define UBRR_57600		12
#define UBRR_115200		6

#elif F_CPU==16000000

#define UBRR_4800		207
#define UBRR_9600		103
#define UBRR_14400		68
#define UBRR_19200		51
#define UBRR_38400		25
#define UBRR_57600		16
#define UBRR_115200		8
#define UBRR_250000		3

#endif

#define UDRE_ON			UCSR0B |= (1<<UDRIE0)
#define UDRE_OFF		UCSR0B &= ~(1<<UDRIE0)

#define BUFSIZE_TX		100
#define BUFSIZE_RX		100
#define DMAXSTRING		12

extern volatile char rxbuf[BUFSIZE_RX];
extern volatile uint8_t rxrdy;

//Enable UART
void uart_init(uint16_t ubrr);

//Send char
void uart_putc(unsigned char data);

//Send string
void uart_puts(const char *s);

//Send decimal number
void uart_putl(long int value, uint8_t radix);

//Send float number
void uart_putd(double value, uint8_t prec);

void uart_nl();

#endif /* _UART_H_ */
