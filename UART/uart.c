/*
 * LittleCircuit FPV Headtracker
 *
 *  Created on: 2016
 * 	    Author: Arkadiusz Artyum Witczak
 *
 * 	ATmega328@16MHz
 * 	Sensors: LSM6DS33, LIS3MDL
 *
 * 	Available under GNU General Public License v3.0
 */

#include "uart.h"

void uart_init(uint16_t ubrr) {
	//Uart baud speed
	UBRR0L = (uint8_t)ubrr;
	UBRR0H = (uint8_t)(ubrr>>8);

	//Enable RX and TX
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

//---------------------------------------------------------------------------------------
//Sender

void uart_putc(unsigned char data) {
    while (!( UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_puts(const char *s) {
    register char c;
    while ((c=*s++)) uart_putc(c);
}

void uart_putl(long int value, uint8_t radix) {
	char string[DMAXSTRING];
	ltoa(value, string, radix);
	uart_puts(string);
}

void uart_putd(double value, uint8_t prec) {
	char string[DMAXSTRING+1];
	dtostrf(value,1,prec,string);
	uart_puts(string);
}

void uart_nl() {
	uart_puts("\r\n");
}

//---------------------------------------------------------------------------------------
//Receiver

volatile char rxbuf[BUFSIZE_RX];
volatile uint8_t rxrdy;

ISR (USART_RX_vect) {
	static uint8_t i;

	char data = UDR0;

	if (rxrdy==0) {
		if ((i>=BUFSIZE_RX) || (data=='$')) i = 0;
		else if (data=='*') {
			rxbuf[i] = '\0';
			rxrdy = 1;
			i = 0;
		}
		else rxbuf[i++] = data;
	}
}
