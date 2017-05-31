#ifndef _ARM_CONTROL_H_
#define _ARM_CONTROL_H_

#include <avr/io.h>
#include "def.h"

#define STEP_PORT PORTA
#define STEP_INTERVAL 100

typedef struct StepperMovement {
  int8_t *intervals;
  int16_t length;
} StepperMovement;

typedef struct ArmPath {
  StepperMovement left;
  StepperMovement right;
} ArmPath;

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

uint8_t start_arm_path(ArmPath *arm_path);
uint8_t is_arm_path_complete();
void stepper_timer();
void init_stepper_state(StepperState *state, StepperMovement *movement);
void move_stepper(StepperState *state);

#endif // _ARM_CONTROL_H_
