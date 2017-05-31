#include "arm_control.h"

ArmPath *current_arm_path = 0;
StepperState left_stepper_state = {
  .travel_mask = 1 << STEP_L,
  .direction_mask = 1 << DIR_L
};
StepperState right_stepper_state = {
  .travel_mask = 1 << STEP_R,
  .direction_mask = 1 << DIR_R
};

// Begin an arm path
uint8_t start_arm_path(ArmPath *arm_path) {
  // Ignore any new arm path until the current one completes
  if (arm_path && !current_arm_path) {
    current_arm_path = arm_path;
    init_stepper_state(&left_stepper_state, &(current_arm_path->left));
    init_stepper_state(&right_stepper_state, &(current_arm_path->right));

    return 1;
  }

  return 0;
}

// Returns whether either arm is currently moving or not
uint8_t is_arm_path_complete() {
  return !left_stepper_state.movement && !right_stepper_state.movement;
}

// Initialize a stepper to begin a movement
void init_stepper_state(StepperState *state, StepperMovement *movement) {
  state->movement = movement;
  state->interval_index = 0;
  state->steps_remaining = 0;
  state->delay_count = 0;
  state->interval_count = 0;
}

// To be called on a timer interrupt to control stepper movement
void stepper_timer(ArmPath *arm_path) {
  if (current_arm_path) {
    move_stepper(&right_stepper_state);
    move_stepper(&left_stepper_state);

    // Check to see if the movements are complete
    if (is_arm_path_complete()) {
      current_arm_path = 0;
    }
  }
}

// Pulse a single stepper based on it's current state
void move_stepper(StepperState *state) {
  // If the path is complete, clear the movement pointer to signal completion
  if (state->movement && state->interval_index >= state->movement->length) {
    state->movement = 0;
  }

  // Nothing to do if there's no movement defined
  if (!state->movement) {
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
