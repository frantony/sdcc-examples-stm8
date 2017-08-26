#include "stm8l.h"

#define TIMER_CLK 16000000

#define MILLISECONDS_PER_SECOND 1000

unsigned int cycle_last;
unsigned int time_ms;

void init_clocksource(void)
{
	/* Set the frequency to 16 MHz */
	CLK_DIVR = 0x00;

	/* Enable TIMER1 clock */
	CLK_PCKENR2 |= CLK_PCKENR2_TIM1;

#define TIM1_DIV (TIMER_CLK / MILLISECONDS_PER_SECOND)

	TIM1_PSCRH = (TIM1_DIV >> 8) & 0xff;
	TIM1_PSCRL = (TIM1_DIV >> 0) & 0xff;

	TIM1_ARRH = 0xff;
	TIM1_ARRL = 0xff;

	/* Enable timer */
	TIM1_CR1 = 0x01;

	cycle_last = 0;
	time_ms = 0;
}

unsigned int read_clocksource(void)
{
	unsigned char h = TIM1_CNTRH;
	unsigned char l = TIM1_CNTRL;

	return ((unsigned int)(h) << 8 | l);
}

/**
 * get_time_ms - get current timestamp in milliseconds
 */
unsigned int get_time_ms(void)
{
	unsigned int cycle_now, cycle_delta;
	unsigned int ms_offset;

	/* read clocksource: */
	cycle_now = read_clocksource();

	/* calculate the delta since the last call: */
	cycle_delta = cycle_now - cycle_last;

	cycle_last = cycle_now;

	/* "convert" to milliseconds */
	ms_offset = cycle_delta;

	time_ms += ms_offset;

	return time_ms;
}

int is_timeout(unsigned int start_ms, unsigned int time_offset_ms)
{
	if ((signed int)(start_ms + time_offset_ms - get_time_ms()) < 0)
		return 1;

	return 0;
}

void mdelay(unsigned int ms)
{
	unsigned int start = get_time_ms();

	while (!is_timeout(start, ms))
		;
}

/*
 * STM8L-DISCOVERY
 *   PC1 --- User Button
 *   PE7 --- Green LED (LD3)
 *   PC7 --- Blue LED (LD4)
 */

#define KEY_PORT_ODR PC_ODR
#define KEY_PORT_DDR PC_DDR
#define KEY_PORT_CR1 PC_CR1
#define KEY_PORT_CR2 PC_CR2
#define KEY_PORT_IDR PC_IDR
#define KEY_PIN BIT(1)

enum key_state {
	KEY_RELEASED = 0x00,
	KEY_PRESSED = 0x01,
};

void init_key(void)
{
	/* direction: input */
	KEY_PORT_DDR &= ~KEY_PIN;
	/* weak pull-up */
	KEY_PORT_CR1 |= KEY_PIN;
	/* disable interrupt */
	KEY_PORT_CR2 &= ~KEY_PIN;
}

enum key_state get_key_state(void)
{
	unsigned char s0;
	unsigned char s1;

	/* debouncing */
	do {
		s0 = KEY_PORT_IDR & KEY_PIN;
		mdelay(50);
		s1 = KEY_PORT_IDR & KEY_PIN;
	} while (s0 != s1);

	if (s0) {
		return KEY_RELEASED;
	}

	return KEY_PRESSED;
}

enum key_event {
	KEY_EVENT_NO_EVENT = 0x00,
	KEY_EVENT_RELEASED = 0x01,
	KEY_EVENT_PRESSED = 0x02,
};

enum key_state last_key_state;

void init_key_event(void)
{
	last_key_state = get_key_state();
}

enum key_event get_key_event(void)
{
	enum key_state cur_state = get_key_state();
	enum key_event ret = KEY_EVENT_NO_EVENT;

	if (last_key_state == cur_state)
		return KEY_EVENT_NO_EVENT;

	if (cur_state == KEY_RELEASED)
		ret = KEY_EVENT_RELEASED;
	else
		if (cur_state == KEY_PRESSED)
			ret = KEY_EVENT_PRESSED;

	last_key_state = cur_state;

	return ret;
}

#define LED_PORT_ODR PE_ODR
#define LED_PORT_DDR PE_DDR
#define LED_PORT_CR1 PE_CR1
#define LED_PIN BIT(7)

enum led_brightness {
	LED_OFF = 0,
	LED_FULL = 255,
};

void init_led(void)
{
	/* direction: output */
	LED_PORT_DDR |= LED_PIN;
	/* push-pull */
	LED_PORT_CR1 |= LED_PIN;
}

void set_led_brightness(enum led_brightness brightness)
{
	unsigned char v;

	v = LED_PORT_ODR;

	if (brightness == LED_OFF) {
		v |= LED_PIN;
	} else if (brightness == LED_FULL) {
		v &= ~LED_PIN;
	}

	LED_PORT_ODR = v;
}

enum led_mode {
	LED_MODE_OFF = 0,
	LED_MODE_ON = 1,
	LED_MODE_BLINK = 2,
	LED_MODE_BLINKFAST = 3,
};

enum led_mode led_mode;

static inline void handle_led(enum led_mode mode)
{
	enum led_brightness brightness;

	brightness = LED_OFF;

	switch (mode) {

	case LED_MODE_BLINK:
		/* blink for 0.6 s every 3 sec */
		if ((read_clocksource() % 3000) > 2400) {
			brightness = LED_FULL;
		}
		break;

	case LED_MODE_BLINKFAST:
		/* blink for 0.1 s every 0.2 sec */
		if ((read_clocksource() % 200) > 100) {
			brightness = LED_FULL;
		}
		break;

	case LED_MODE_ON:
		brightness = LED_FULL;
		break;

	case LED_MODE_OFF:
	default:
		brightness = LED_OFF;
	}

	set_led_brightness(brightness);
}

static inline void putc_ll(char value)
{
	while (!(USART1_SR & USART_SR_TXE))
		;

	USART1_DR = value;
}

static void puts_ll(const char * str)
{
	while (*str) {
		if (*str == '\n')
			putc_ll('\r');

		putc_ll(*str);
		str++;
	}
}

static void puthex_ll(unsigned int value)
{
	int i;
	unsigned char ch;

	for (i = 4; i--; ) {
		ch = ((value >> (i * 4)) & 0xf);
		ch += (ch >= 10) ? 'a' - 10 : '0';
		putc_ll(ch);
	}
}

int getc_ll(void)
{
	if (!((USART1_SR) & USART_SR_RXNE))
		return -1;

	return USART1_DR;
}

static inline void init_uart(void)
{
	/* Enable USART1 clock */
	CLK_PCKENR1 |= CLK_PCKENR1_UART1;

	/* Map USART1 to PA2[TX] and PA3[RX] */
	SYSCFG_RMPCR1 = 0x10;

	PA_DDR = 0x04; // Setup PA2[USART1_TX] as output
	PA_CR1 = 0x04; // Setup PA2[USART1_TX] as push-pull output

	/* Allow TX & RX */
	USART1_CR2 = USART_CR2_TEN | USART_CR2_REN;

	/* 1 stop bit */
	USART1_CR3 &= ~(USART_CR3_STOP1 | USART_CR3_STOP2);

	/* N.B. better value can be obtained with rounding! */
#define USART1_DIV (TIMER_CLK / 115200)

	/*
	 * See the 'UART register map and reset values' table
	 * in the STM8S Series and STM8AF Series 8-bit microcontrollers
	 * Reference manual RM0016 for details.
	 *
	 *     USART1_BRR2 = { UART_DIV[15:12], UART_DIV[3:0] };
	 *     USART1_BRR1 = { UART_DIV[11:4] };
	 */

	USART1_BRR2 = ((USART1_DIV >> 12) & 0xf) | ((USART1_DIV >> 0) & 0xf);
	USART1_BRR1 = (USART1_DIV >> 4) & 0xff;
}

void debug_message(const char *message)
{
	puthex_ll(read_clocksource());
	puts_ll(": ");
	puts_ll(message);
	puts_ll("\n");
}

enum fsm_state {
	INITIAL = 0,
	STANDBY = 1,
	STANDBYFIN = 2,
	WORKING = 3,
	WORKINGFIN = 4,
};

enum fsm_state fsm_state;
unsigned int lastevent;

void fsm_next_state(enum fsm_state next_state)
{
	switch (next_state) {
	case INITIAL:
		led_mode = LED_MODE_OFF;
		break;

	case STANDBY:
		led_mode = LED_MODE_BLINK;
		break;

	case STANDBYFIN:
		led_mode = LED_MODE_BLINKFAST;
		break;

	case WORKING:
		led_mode = LED_MODE_ON;
		break;

	case WORKINGFIN:
		led_mode = LED_MODE_BLINKFAST;
		break;
	}

	fsm_state = next_state;
	lastevent = read_clocksource();
}

void fsm_tick(void)
{
	enum key_event key_event = KEY_EVENT_NO_EVENT;

	switch (fsm_state) {
	case INITIAL:
		puts_ll("\n");
		debug_message("INITIAL");
		fsm_next_state(STANDBY);
		get_key_event();
		break;

	case STANDBY:
		debug_message("STANDBY");
		key_event = get_key_event();
		if (key_event == KEY_EVENT_PRESSED) {
			fsm_next_state(STANDBYFIN);
		}
		break;

	case STANDBYFIN:
		debug_message("STANDBYFIN");
		get_key_event();
		if (is_timeout(lastevent, 1000)) {
			fsm_next_state(WORKING);
		}
		break;

	case WORKING:
		debug_message("WORKING");
		key_event = get_key_event();
		if (key_event == KEY_EVENT_PRESSED) {
			fsm_next_state(WORKINGFIN);
		}
		break;

	case WORKINGFIN:
		debug_message("WORKINGFIN");
		key_event = get_key_event();
		if (key_event == KEY_EVENT_RELEASED) {
			fsm_next_state(WORKING);
			break;
		}

		if (is_timeout(lastevent, 3000)) {
			fsm_next_state(STANDBY);
		}
		break;

	default:
		fsm_next_state(INITIAL);
		break;
	}
}

void main(void)
{
	/* Disable all peripheral clocks */
	CLK_PCKENR1 = 0x0;
	CLK_PCKENR2 = 0x0;

	init_clocksource();

	init_uart();

	init_led();

	init_key();
	init_key_event();

	fsm_next_state(INITIAL);

	for(;;) {
		int ci;

		fsm_tick();

		ci = getc_ll();
		if (ci != -1) {
			unsigned char c;

			c = ci;
			switch (c) {
			case 'R':
				fsm_next_state(INITIAL);
				break;

			case 'S':
				fsm_next_state(STANDBY);
				break;

			case 'W':
				fsm_next_state(STANDBYFIN);
				break;
			default:
				/* do nothing */
			}
		}

		handle_led(led_mode);
	}
}
