//2017 - 05 - 16
#ifndef _MOTOR_FUNCTIONS_H_
#define _MOTOR_FUNCTIONS_H_

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>
#include "def.h"

//PID stuff was found online
//should be float but that doesn't work on the 128
typedef struct {
  int16_t max; //max manipulated value
  int16_t min; //min manupulated calue
  
  //consts (divide by 10,000)
  int16_t proportional_const; 	//const
  int16_t intergral_const;	//const
  int16_t derivative_const;	//const
  
  //these change
  int16_t error;
  int16_t intergral_value;
  //int16_t control;
} PID;

void pid_init(PID *pid, int16_t min, int16_t max);
int16_t pid_update(PID* pid, int16_t set_point, int16_t process_var);

/*
typedef struct {
  int16_t windup_guard;
  int16_t proportional_gain;
  int16_t intergral_gain;
  int16_t derivative_gain;
  int16_t prev_error;
  int16_t int_error;
  int16_t control;
} PID;

void pid_update(PID* pid, int16_t curr_error, int16_t dt);
*/

void z_up();
void z_down();
void z_halt();

void solinoid_cw();
void solinoid_ccw();
void solinoid_halt();

#endif // _MOTOR_FUNCTIONS_H_
