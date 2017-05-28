//2017 - 05 - 16
#ifndef _MOTOR_FUNCTIONS_H_
#define _MOTOR_FUNCTIONS_H_

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>
#include "def.h"

void solinoid_cw();
void solinoid_ccw();
void solinoid_halt();

#endif // _MOTOR_FUNCTIONS_H_
