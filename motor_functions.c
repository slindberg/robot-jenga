//2017 - 05 - 16

#include "motor_functions.h"
#define STEP_PORT PORTA
#define STEP_INTERVAL 100

void move_stepper(StepperState *state) {
  if (!state->movement || state->interval_index >= state->movement->length) {
    return;
  }

  // Clear the travel pin to reset pulse state
  STEP_PORT &= ~state->travel_mask;

  // If we're at the start of an interval, initialize remaining steps
  if (state->interval_count == 0) {
    int8_t delta = state->movement->intervals[state->interval_index];

    // Set direction pin, high is clockwise
    if (delta < 0) {
      STEP_PORT |= state->direction_mask;
      state->steps_remaining = -delta;
    } else {
      STEP_PORT &= ~state->direction_mask;
      state->steps_remaining = delta;
    }
  }

  if (state->steps_remaining != 0) {
    // If the call delay is zero, we've just completed a step, so calculate a new
    // delay. This is done to ensure that all steps are completed within the interval,
    // since integer division would result in missed steps.
    if (state->call_delay == 0) {
      state->call_delay = (STEP_INTERVAL - state->interval_count) / state->steps_remaining;
    }

    // If the enough timer calls have been made, pulse the stepper and reset state vars
    if (state->delay_count == state->call_delay) {
      STEP_PORT |= state->travel_mask;
      state->steps_remaining--;
      state->call_delay = 0;
      state->delay_count = 0;
    } else {
      state->delay_count++;
    }
  }

  // At the end of an interval, reset counts and advance to the next interval,
  if (state->interval_count == STEP_INTERVAL) {
    state->interval_index++;
    state->interval_count = 0;
    state->delay_count = 0;
  } else {
    state->interval_count++;
  }
}

void reset_stepper_state(StepperState *state, StepperMovement *movement) {
    state->movement = movement;
    state->interval_index = 0;
    state->steps_remaining = 0;
    state->delay_count = 0;
    state->interval_count = 0;
}
