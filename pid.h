#ifndef PID2_H
#define PID2_H
#include <math.h>
#include <avr/io.h>
#define DEFAULT_MAXERROR 10e8
#define DEFAULT_MAXSUMERROR 10e12

typedef struct PID_DATA{
	float lastValue;
	float Kp;
	float Ki;
	float Kd;
	float sumError;
	float maxError;
	float maxSumError;
	int32_t minOutput;
	int32_t maxOutput;
} PID_DATA;

void initializePID(PID_DATA *pid, float p, float i, float d, int32_t minOutput, int32_t maxOutput);

//step takes about 500us on an atmega8 at 20 Mhz
int32_t stepPID(PID_DATA *pid, float value, float reference);

void resetSumError(PID_DATA *pid);

#endif
