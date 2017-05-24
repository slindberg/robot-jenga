//2017 - 05 - 16

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>

typedef struct StepperState {
  uint8_t travel_mask;
  uint8_t direction_mask;
  int8_t *step_intervals;
  uint16_t total_intervals;
  uint16_t interval_index;
  uint8_t interval_count;
  uint8_t steps_remaining;
  uint8_t call_delay;
  uint8_t delay_count;
} StepperState;

void stepper(StepperState *state);
