#include "ef_rotation_control.h"

pid_params_t ef_pid_params = {
	.kp = 1.0,
	.ki = 0.0,
	.kd = 0.0,
	.min_output = 2000,
	.max_output = 2100,
};

pid_state_t ef_pid_state;

int32_t ef_angle = 0; //reference angle
int32_t ef_set_point = 0; //where to go
uint16_t ef_duty = 0; //pwm duty cycle

int32_t get_ef_angle() {
  return ef_angle;
}

int32_t get_ef_set_point() {
  return ef_set_point;
}

uint16_t get_ef_duty() {
  return ef_duty;
}

pid_params_t *get_ef_pid_params() {
  return &ef_pid_params;
}

void init_ef_pid() {
  init_pid_state(&ef_pid_state, &ef_pid_params);
}

void rotate_ef(int8_t delta) {
  rotate_ef_absolute(ef_angle + delta);
}

void rotate_ef_absolute(int16_t set_point) {
  ef_set_point = set_point;
}

uint16_t process_ef_rotation() {
  static uint8_t count = 0;

  if (ef_set_point == ef_angle) {
    ef_halt();
    ef_duty = 0;
    reset_pid_error(&ef_pid_state);
  } else {
    if (ef_set_point > ef_angle) {
      set_pid_direction(&ef_pid_state, PID_DIRECT);
      ef_ccw();
    } else {
      set_pid_direction(&ef_pid_state, PID_REVERSE);
      ef_cw();
    }

    if (count == 0) {
      ef_duty = step_pid(&ef_pid_state, ef_angle, ef_set_point);
    }
    count++;
  }

  return ef_duty;
}

void ef_rotation_encoder() {
  if (bit_is_set(PINF, SE2_PIN)) {
    ef_angle++;
  } else {
    ef_angle--;
  }
}

void ef_cw() {
  PORTF &= ~(1<<SL);
  PORTF |= (1<<SR);
}

void ef_ccw() {
  PORTF &= ~(1<<SR);
  PORTF |= (1<<SL);
}

void ef_halt() {
  PORTF &= ~((1<<SR) | (1<<SL));
}
