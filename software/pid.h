/*
 * pid.h
 *
 * Created: 13.05.2020 19:41:56
 *  Author: Emre Emir Fidan
 */ 


#ifndef PID_H_
#define PID_H_

double *input;
double *output;
double *setpoint;
double kp;
double ki;
double kd;
int SampleTime;
double min;
double max;

unsigned long lastTime;
double LastInput;

extern void set_pid(double*, double*, double*, double, double, double, int, double, double);

extern void compute();


#endif /* PID_H_ */