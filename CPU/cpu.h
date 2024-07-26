#ifndef CPU_CPU_H_
#define CPU_CPU_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../FUNC/func.h"
#include "../BTN/btn.h"
#include "../UART/uart.h"

//PPM OUT
#define PPM_DDR DDRB
#define PPM_PORT PORTB
#define PPM_PIN PB2

//LED
#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PIN PD5

//Button
#define BTN_DDR DDRB
#define BTN_PORT PORTB
#define BTN_PINPORT PINB
#define BTN_PIN PB0

/////////////////////////////////////////////////////////////////////////////////////////////////
//Macros
#define PIN_LOW(PORT,PIN) PORT &= ~(1<<PIN)
#define PIN_HIGH(PORT,PIN) PORT |= (1<<PIN)
#define PIN_TOG(PORT,PIN) PORT ^= (1<<PIN)

#define LED_ON PIN_HIGH(LED_PORT,LED_PIN)
#define LED_OFF PIN_LOW(LED_PORT,LED_PIN)
#define LED_TOG PIN_TOG(LED_PORT,LED_PIN)

#define READ_PIN(PINPORT,PIN) (PINPORT & (1<<PIN))

extern volatile uint16_t t_resetpos;
extern volatile uint16_t t_ms;

void cpu_init(void);

#endif /* CPU_CPU_H_ */
