//2017 - 05 - 16
#ifndef _MOTOR_FUNCTIONS_H_
#define _MOTOR_FUNCTIONS_H_

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>

typedef struct StepperMovement {
  int8_t *intervals;
  int16_t length;
} StepperMovement;

typedef struct StepperState {
  uint8_t travel_mask;
  uint8_t direction_mask;
  StepperMovement *movement;
  uint16_t interval_index;
  uint8_t interval_count;
  uint8_t steps_remaining;
  uint8_t call_delay;
  uint8_t delay_count;
} StepperState;

void move_stepper(StepperState *state);

void reset_stepper_state(StepperState *state, StepperMovement *movement);

#endif // _MOTOR_FUNCTIONS_H_
