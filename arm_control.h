#ifndef _ARM_CONTROL_H_
#define _ARM_CONTROL_H_

#include <avr/io.h>
#include "def.h"

#define STEP_PORT PORTA
#define STEP_INTERVAL 100

typedef struct stepper_movement_t {
  int8_t *intervals;
  int16_t length;
} stepper_movement_t;

typedef struct arm_path_t {
  stepper_movement_t left;
  stepper_movement_t right;
} arm_path_t;

typedef struct stepper_state_t {
  uint8_t travel_mask;
  uint8_t direction_mask;
  stepper_movement_t *movement;
  uint16_t interval_index;
  uint8_t interval_count;
  uint8_t steps_remaining;
  uint8_t call_delay;
  uint8_t delay_count;
} stepper_state_t;

uint8_t start_arm_path(arm_path_t *arm_path);
uint8_t is_arm_path_complete();
void stepper_timer();
void init_stepper_state(stepper_state_t *state, stepper_movement_t *movement);
void move_stepper(stepper_state_t *state);

#endif // _ARM_CONTROL_H_
