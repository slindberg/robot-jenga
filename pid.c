// Adapted from https://github.com/pengumc/avr_simple_pid
// Author: Michiel van der Coelen

#include <avr/io.h>
#include <math.h>
#include "pid.h"

void init_pid_state(pid_state_t *state, pid_params_t *params) {
  state->kp = params->kp;
  state->ki = params->ki;
  state->kd = params->kd;
	state->last_input = 0;
	state->sum_error = 0;
  state->direction = PID_DIRECT;
  state->params = params;
}

int32_t step_pid(pid_state_t *state, int32_t input, int32_t set_point) {
	float error, p_term, i_term, d_term;
  pid_params_t *params = state->params;

	error = (float)(set_point - input);

	p_term = error * state->kp;

	state->sum_error += error;
	i_term = state->sum_error * state->ki;
	if (i_term > params->max_output) i_term = params->max_output;
	if (i_term < params->min_output) i_term = params->min_output;

	d_term = (input - state->last_input) * state->kd;
	state->last_input = input;

	int32_t output = (int32_t)(p_term + i_term - d_term);
	if (output > params->max_output) output = params->max_output;
  if (output < params->min_output) output = params->min_output;

  return output;
}

void set_pid_direction(pid_state_t *state, pid_dir_t direction) {
  if (state->direction != direction) {
    state->kp = -state->kp;
    state->ki = -state->ki;
    state->kd = -state->kd;
    state->direction = direction;
  }
}

void reset_pid_error(pid_state_t *state) {
  state->sum_error = 0;
}
