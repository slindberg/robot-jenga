#ifndef ZAXIS_H
#define ZAXIS_H

#include <avr/io.h>
#include "def.h"

//about 100 revs * 1000 counts

void move_zaxis(int32_t des_pos);
void move_zaxis_absolute(int32_t des_pos);

void z_up();
void z_down();
void z_halt();
void zaxis_limit_switch();
void zaxis_encoder();
uint16_t process_zaxis();
void home_zaxis();

  //PID stuff was found online
//should be float but that doesn't work on the 128
typedef struct {
  int16_t max; //max manipulated value
  int16_t min; //min manupulated calue
  
  //consts (divide by 10,000)
  float proportional_const; 	//const
  float intergral_const;	//const
  float derivative_const;	//const
  
  //these change
  float error;
  float intergral_value;
  //int16_t control;
} PID;

void zpid_init();
uint16_t pid_update(PID* pid, float set_point, float process_var);

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

#endif
