#include "btn.h"

volatile uint16_t t_btn;
uint8_t btn_status;

void key_check(uint8_t *status, volatile uint16_t *timer, uint8_t pinport, uint8_t pin) {

	//Button presed
	if (BTN_PRESSED(pinport,pin)) {
		if (*status==0)	{ *status=1; *timer=0; } //1 - Initial press
		else if (*status==1 && *timer>=KEYPRESS_SHORT) { *status=2; *timer=0; } //2 - Short press single (key pressed)
		else if (*status==2) *status=3; //3 - Short press continuous
		else if (*status==3 && *timer>=KEYPRESS_LONG) { *status=4; *timer=0; } //Long press single
		else if (*status==4) *status=5; //5 - ong press continuous
	}

	//Button released
	else if (*status!=0) {
		if (*status==1) *status=0;
		else if (*status==2 || *status==3) { *status=11; *timer=0; } //11 - Short press single (key released)
		else if (*status==11) *status=0; //Exit from short press
		else if (*status==4 || *status==5) { *status=14; *timer=0; } //14 - Long press single (key released)
		else if (*status==14) *status=0; //Exit from long press
	}
}
