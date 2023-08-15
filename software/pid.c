/*
 * pid.c
 *
 * Created: 13.05.2020 19:41:43
 *  Author: Emre Emir Fidan
 */ 

#include "pid.h"
#include "timers.h"

void set_pid(double* in, double* out, double* sp, double p, double i, double d, int st, double mn, double mx) {
	input = in;
	output = out;
	setpoint = sp;
	SampleTime = st;
	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = p;
	ki = i * SampleTimeInSec;
	kd = d / SampleTimeInSec;
	min = mn;
	max = mx;
	lastTime = millis() - SampleTime;
}

void compute() {
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if (timeChange >= SampleTime) {
		double error = *setpoint - *input;
		double ITerm = *input;
		ITerm += (ki * error);
		if (ITerm > max) ITerm = max;
		else if (ITerm < min) ITerm = min;
		
		double dinput = (*input - LastInput);
		double out = kp * error + ITerm - kd * dinput;
		
		if (out > max) out = max;
		else if (out < min) out = min;
		
		*output = out;
		
		LastInput = *input;
		lastTime = now;
	}
}