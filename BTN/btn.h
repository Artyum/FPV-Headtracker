#ifndef BTN_H_
#define BTN_H_

#include <avr/io.h>

//Statusy przycisku
/* KEY_DN_SHORT_S  - The moment you press the button - short click
 * KEY_DN_SHORT_C  - Button pressed - short click
 * KEY_DN_LONG_S   - Start of a long click
 * KEY_DN_LONG_C   - Button pressed - long click
 *
 * KEY_UP          - Key released
 * KEY_UP_SHORT    - Button released after a short click
 * KEY_UP_LONG     - Button released after a long click
 */

#define KEY_UP				0
#define KEY_DN_SHORT_S		2
#define KEY_DN_SHORT_C		3
#define KEY_DN_LONG_S		4
#define KEY_DN_LONG_C		5
#define KEY_UP_SHORT		11
#define KEY_UP_LONG			14

#define KEYPRESS_SHORT		20
#define KEYPRESS_LONG		700
#define KEYPRESS_REPEAT		1300
#define KEYDBL_BREAK_TIME	250

#define BTN_PRESSED(PINPORT,PIN) (!(PINPORT & (1<<PIN)))

extern volatile uint16_t t_btn;
extern uint8_t btn_status;

void key_check(uint8_t *keystatus, volatile uint16_t *timer, uint8_t pinport, uint8_t pin);

#endif /* BTN_H_ */
