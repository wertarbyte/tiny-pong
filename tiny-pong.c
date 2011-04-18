#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define EEPROM_SEED_LOCATION 0

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define ROWS 7
#define COLS 5

#define MAX_X (ROWS-1)
#define MAX_Y (COLS-1)

#define BTN_L PA0
#define BTN_R PA1

const static uint8_t COL_PIN[COLS] = {
	PD0,
	PD1,
	PD2,
	PD3,
	PD4,
};

const static uint8_t ROW_PIN[ROWS] = {
	PB6,
	PB5,
	PB3,
	PB4,
	PB2,
	PB1,
	PB0
};

static volatile struct {
	uint8_t xpos;
	uint8_t ypos;
	int8_t dx;
	int8_t dy;
} ball;

struct paddle {
	const uint8_t row;
	uint8_t pos;
	uint8_t last_pos;
	const uint8_t width;
	uint8_t score;
	uint8_t target;
	uint8_t ai;
};


static uint8_t serving = 0;

static volatile struct paddle player[2] = {
	{     0, MAX_Y/2, MAX_Y/2, 2, 0, MAX_Y/2, 1},
	{ MAX_X, MAX_Y/2, MAX_Y/2, 2, 0, MAX_Y/2, 1},
};

static void start(void) {
	int8_t direction = -1 * (serving*2 - 1);
	ball.xpos = player[serving].row+direction;
	ball.ypos = player[serving].pos;
	ball.dx = direction;
	ball.dy = (random()%2)*2 - 1;

	serving = (serving+1)%2;
}

static void init(void) {
	for (int i=0; i<COLS; i++) {
		set_output(DDRD, COL_PIN[i]);
		output_high(PORTD, COL_PIN[i]);
	}
	for (int i=0; i<ROWS; i++) {
		set_output(DDRB, ROW_PIN[i]);
		output_low(PORTB, ROW_PIN[i]);
	}
#ifdef MANUAL_CONTROL
	set_input(DDRA, BTN_L);
	set_input(DDRA, BTN_R);
#endif
	// seed random number generator
	uint8_t seed = eeprom_read_byte(EEPROM_SEED_LOCATION);
	srandom(seed);
	eeprom_update_byte(EEPROM_SEED_LOCATION, seed+1);
	start();

	OCR1A = 2;
	TCCR1A = 0x00;
	// WGM1=4, prescale at 1024
	TCCR1B = (0 << WGM13)|(1 << WGM12)|(1 << CS12)|(0 << CS11)|(1 << CS10);
	//Set bit 6 in TIMSK to enable Timer 1 compare interrupt
	TIMSK |= (1 << OCIE1A);
	sei();
}

static uint8_t pixel_on(uint8_t y, uint8_t x) {
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

static volatile uint8_t active_col = 0;

static void draw_screen(void) {
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

static int8_t aim(uint8_t x) {
	int8_t dist_x = x - ball.xpos;
	// how many ticks until it intersects?
	int8_t t = dist_x/ball.dx;
	if (t<0) {
		// the ball is moving away from the paddle
		return -1;
	}
	// where will it be on the y scale then?
	int8_t y = ball.ypos + (t * ball.dy);
	// simulate the mirroring at the border
	while (y < 0 || y > MAX_Y) {
		if (y < 0) y = abs(y);
		else if (y > MAX_Y) y = (MAX_Y-(y-MAX_Y));
	}
	return y;
}

static uint8_t ball_lost(void) {
	return ((ball.xpos + ball.dx) < 0 || (ball.xpos + ball.dx) >= ROWS);
}

static uint8_t bounce_ball(void) {
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
			/* transfer impulse from paddle to ball */
			ball.dy += (p->pos - p->last_pos);
			return 1;
		}
	}
	return 0;
}

static int8_t min(int8_t a, int8_t b) {
	return a < b ? a : b;
}
static int8_t max(int8_t a, int8_t b) {
	return a > b ? a : b;
}

static void move_paddles(void) {
	for (uint8_t i=0; i<2; i++) {
		volatile struct paddle *p = &player[i];
		p->last_pos = p->pos;
#ifdef MANUAL_CONTROL
		if (!p->ai) {
			// manual movement
			if (PINA & (1<<BTN_L)) {
				p->pos = max(0, min(MAX_Y+1 - p->width, p->pos-1));
			} else if (PINA & (1<<BTN_R)) {
				p->pos = max(0, min(MAX_Y+1 - p->width, p->pos+1));
			}

			continue;
		}
#endif
		if (p->pos == p->target) {
			/* select a new target */
			int8_t dist = abs(ball.xpos - p->row);
			if (dist <= 2) {
				int8_t target = aim(p->row);
				if (target >= 0) {
					/*
					if (random()%2 == 0) {
						target += ball.dy;
					}
					*/
					p->target = max(0, min(MAX_Y+1 - p->width, target));
				}
			} else if(dist > MAX_X-2) {
				p->target = random()%(COLS-p->width);
			}
		} else {
			/* move towards our target */
			if (p->target > p->pos) p->pos++;
			else p->pos--;
		}
	}
}

static void move_ball(void) {
	while (bounce_ball());
	if (ball_lost()) {
		_delay_ms(500);
		start();
	} else {
		ball.xpos += ball.dx;
		ball.ypos += ball.dy;
	}
}

int main(void) {
	init();

	while(1) {
#ifdef MANUAL_CONTROL
		// see if someone wants to take over a player
		if (player[0].ai && player[1].ai) {
			// manual movement
			if (PINA & (1<<BTN_R)) {
				player[0].ai = 0;
				player[1].ai = 1;
			} else if (PINA & (1<<BTN_L)) {
				player[0].ai = 1;
				player[1].ai = 0;
			}
		}
#endif
		_delay_ms(200);
		move_ball();
		move_paddles();
	}
	return 0;
}
