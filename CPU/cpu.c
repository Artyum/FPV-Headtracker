#include "cpu.h"

volatile uint16_t t_resetpos;
volatile uint16_t t_ms;

void cpu_init(void) {
	//OUT PPM
	PIN_HIGH(PPM_DDR,PPM_PIN);		//OC1B
	PIN_HIGH(PPM_PORT,PPM_PIN);

	//Timer2 8-bit
	TCCR2A = (1<<WGM21);			//CTC
	TCCR2B = (1<<CS22);				//64 Prescaler
	OCR2A = 250;					//Int every 1ms
	TIMSK2 = (1<<OCIE2A);

	//Timer1 16-bit
	TCCR1A = (1<<COM1B1)|(1<<WGM11)|(1<<WGM10);		//Fast PWM mode 15; non-inverting mode
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);		//8 prescaler (microseconds)
	TIMSK1 = (1<<OCIE1A);							//CompareMatch on OCR1A
	OCR1A = 5;

	//LED OUT
	PIN_HIGH(LED_DDR,LED_PIN);
	LED_OFF;

	//Button IN
	PIN_LOW(BTN_DDR,BTN_PIN);
	PIN_HIGH(BTN_PORT,BTN_PIN); //Pull-up

	sei();
}

ISR(TIMER2_COMPA_vect) {
	uint16_t n;

	t_ms++;
	t_btn++;

	n = t_resetpos;
	if (n) {
		if (n==1) LED_OFF;
		t_resetpos = --n;
	}
}

//PPM Generation
ISR(TIMER1_COMPA_vect) {
	//Channels lengths
	static uint16_t ch_len_isr[CH_NUMBER];

	//Current channel
	static uint8_t ch_num;

	//PPM frame remaining time
	static uint16_t ch_fill;

	//OCR1A - TOP
	//OCR1B - DUTY

	if (ch_num < CH_NUMBER) {
		//Channel
		uint16_t len = ch_len_isr[ch_num];
		OCR1A = len + PPM_DELAY;
		OCR1B = len;
		ch_fill += (len + PPM_DELAY);
		ch_num++;
	}
	else {
		//PPM fill
		uint16_t len = PPM_LEN - ch_fill;
		OCR1A = len;
		OCR1B = len - PPM_DELAY;
		ch_num = 0;
		ch_fill = 0;

		//Get new channels lengths
		for (uint8_t i=0; i<CH_NUMBER; i++) ch_len_isr[i]=ch_len[i];
	}
}
