#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define ROWS 7
#define COLS 5

const uint8_t COL_PIN[COLS] = {
	PD0,
	PD1,
	PD2,
	PD3,
	PD4,
};

const uint8_t ROW_PIN[ROWS] = {
	PB0,
	PB1,
	PB2,
	PB4,
	PB3,
	PB5,
	PB6
};

struct paddle {
	const uint8_t row;
	uint8_t pos;
	uint8_t width;
	uint8_t score;
};

struct paddle player[2] = {
	{ 0, 0, 2, 0 },
	{ ROWS-1, 0, 2, 0 },
};

void init(void) {
	for (int i=0; i<COLS; i++) {
		set_output(DDRD, COL_PIN[i]);
		output_high(PORTD, COL_PIN[i]);
	}
	for (int i=0; i<ROWS; i++) {
		set_output(DDRB, ROW_PIN[i]);
		output_low(PORTB, ROW_PIN[i]);
	}
	set_input(DDRA, PA0);
	set_input(DDRA, PA1);
	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);
	sei();
}

inline uint8_t pixel_on(uint8_t y, uint8_t x) {
	for (uint8_t i=0; i<2; i++) {
		struct paddle *p = &player[i];
		if ( x == p->row && y >= p->pos && y < (p->pos + p->width)) {
			return 1;
		}
	}
	return 0;
}

volatile uint8_t active_col = 0;
void inline draw_screen(void) {
	output_low(PORTD,COL_PIN[ (COLS+active_col-1)%COLS ]);
	for (int x=0; x<ROWS; x++) {
		if (pixel_on(active_col, x)) {
			output_low(PORTB, ROW_PIN[x]);
		} else {
			output_high(PORTB, ROW_PIN[x]);
		}
	}
	output_high(PORTD,COL_PIN[active_col]);
	active_col = (active_col+1)%COLS;
}

SIGNAL(SIG_TIMER1_COMPA) {
	draw_screen();
}

int main(void) {
	init();

	while(1) {
		_delay_ms(250);
	}
	return 0;
}
