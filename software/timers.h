/*
 * timers.h
 *
 * Created: 13.05.2020 20:02:38
 *  Author: Emre Emir Fidan
 */ 

#include <avr/io.h>

#ifndef TIMERS_H_
#define TIMERS_H_

#define PWM_MAX 255
#define PWM_MIN 0

extern void enable_timer0();
volatile unsigned long msec;
extern unsigned long millis();

extern void enable_timer1();
extern void set_pwm(uint16_t ms);


#endif /* TİMERS_H_ */