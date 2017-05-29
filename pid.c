//author: Michiel van der Coelen
//date 2010-10-10

#include "pid.h"
#include <avr/io.h>
#include <math.h>
void initializePID(PID_DATA *pid, float p, float i, float d, int32_t minOutput, int32_t maxOutput){
	pid->lastValue = 0;
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
	pid->sumError = 0;
	pid->maxError = DEFAULT_MAXERROR;
	// if(i) pid->maxSumError = DEFAULT_MAXSUMERROR/i;
	// else pid->maxSumError = DEFAULT_MAXSUMERROR;
  pid->maxSumError = (float)maxOutput;
  pid->maxOutput = maxOutput;
}

int32_t stepPID(PID_DATA *pid, float value, float reference){
	float error, temp, pFactor, iFactor, dFactor;

	error = reference - value;
	if(error > pid->maxError) error = pid->maxError;
	if(error < -pid->maxError) error = -pid->maxError;

	//P
	pFactor = error * pid->Kp;

	//I
	temp = pid->sumError + error;
	if(temp > pid->maxSumError) temp = pid->maxSumError;
	if(temp < -pid->maxSumError) temp = -pid->maxSumError;
	pid->sumError = temp;
	iFactor = pid->sumError * pid->Ki;

	//D
	dFactor = (value - pid->lastValue)*pid->Kd;
	pid->lastValue = value;

	temp = pFactor + iFactor + dFactor;

	int32_t result = ((int32_t) temp);

	if(result > pid->maxOutput) result = pid->maxOutput;
  if(result < pid->minOutput) result = pid->minOutput;

  return result;
}

void resetSumError(PID_DATA *pid){
	pid->sumError =0;
	pid->maxSumError = DEFAULT_MAXSUMERROR/pid->Ki;
}
