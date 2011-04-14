#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

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
	PB6,
	PB5,
	PB3,
	PB4,
	PB2,
	PB1,
	PB0
};

volatile struct {
	uint8_t xpos;
	uint8_t ypos;
	int8_t dx;
	int8_t dy;
} ball;

struct paddle {
	const uint8_t row;
	uint8_t pos;
	uint8_t width;
	uint8_t score;
	uint8_t target;
};


uint8_t serving = 0;

volatile struct paddle player[2] = {
	{ 0, 0, 2, 0, 0},
	{ ROWS-1, 0, 2, 0, 0 },
};

void start(void) {
	int8_t direction = -1 * (serving*2 - 1);
	ball.xpos = player[serving].row+direction;
	ball.ypos = player[serving].pos;
	ball.dx = direction;
	ball.dy = (random()%2)*2 - 1;

	serving = (serving+1)%2;
}

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
	start();

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
		volatile struct paddle *p = &player[i];
		if ( x == p->row && y >= p->pos && y < (p->pos + p->width)) {
			return 1;
		}
	}
	if ( x == ball.xpos && y == ball.ypos ) {
		return 1;
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

inline uint8_t ball_lost(void) {
	return ((ball.xpos + ball.dx) < 0 || (ball.xpos + ball.dx) >= ROWS);
}

inline uint8_t bounce_ball(void) {
	int8_t nx = ball.xpos + ball.dx;
	int8_t ny = ball.ypos + ball.dy;
	/* collision with upper/lower border */
	if (ny < 0 || ny > COLS-1) {
		ball.dy *= -1;
		return 1;
	}
	/* collision with a paddle */
	for (uint8_t i=0; i<2; i++) {
		volatile struct paddle *p = &player[i];
		if ( nx == p->row && ny >= p->pos && ny < (p->pos + p->width)) {
			ball.dx *= -1;
			/* did we hit the edge? */
			if ( ball.ypos > (p->pos + p->width-1) || ball.ypos < p->pos ) {
				ball.dy *= -1;
			}
			return 1;
		}
	}
	return 0;
}

inline void move_ball(void) {
	ball.xpos += ball.dx;
	ball.ypos += ball.dy;
}

inline void move_paddles(void) {
	for (uint8_t i=0; i<2; i++) {
		volatile struct paddle *p = &player[i];
		if (p->pos == p->target) {
			/* select a new target */
			p->target = random()%(COLS-p->width);
		} else {
			/* move towards our target */
			if (p->target > p->pos) p->pos++;
			else p->pos--;
		}
	}
}

inline void move(void) {
	while (bounce_ball());
	if (ball_lost()) {
		_delay_ms(500);
		start();
	} else {
		move_ball();
	}
	move_paddles();
}

int main(void) {
	init();

	while(1) {
		_delay_ms(250);
		move();
	}
	return 0;
}
