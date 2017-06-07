#ifndef PID_H
#define PID_H

#include <math.h>
#include <avr/io.h>
#define PID_DIRECT 0
#define PID_REVERSE 1

typedef int8_t pid_dir_t;

typedef struct pid_params_t {
	float kp;
	float ki;
	float kd;
	int32_t min_output;
	int32_t max_output;
} pid_params_t;

typedef struct pid_state_t {
	float kp;
	float ki;
	float kd;
	float sum_error;
	int32_t last_input;
  pid_dir_t direction;
  pid_params_t *params;
} pid_state_t;

void init_pid_state(pid_state_t *state, pid_params_t *params);

int32_t step_pid(pid_state_t *state, int32_t input, int32_t set_point);

void set_pid_direction(pid_state_t *state, pid_dir_t direction);

void reset_pid_error(pid_state_t *state);

#endif
