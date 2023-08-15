/*
 * timers.c
 *
 * Created: 13.05.2020 19:59:27
 *  Author: Emre Emir Fidan
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timers.h"

void enable_timer0() {
	TIMSK |= (1 << TOIE0);
	sei();
	TCCR0 |= (1 << CS01) | (1 << CS00);
}

unsigned long millis() {
	return msec * 2;
}

void enable_timer1() {
	DDRB |= (1 << PORTB1) | (1 << PORTB2);
	OCR1A = 0x0080;
	TCCR1A |= (1 << COM1A1) | (1 << WGM10);
	TCCR1B |= (1 << WGM12) | (1 << CS10);
}

void set_pwm(uint16_t ms) {
	if (ms > PWM_MAX) ms = PWM_MAX;
	else if (ms < PWM_MIN) ms = PWM_MIN;
	OCR1A = ms;
}

ISR (TIMER0_OVF_vect)
{
	TCNT0 += 6;
	msec++;
}