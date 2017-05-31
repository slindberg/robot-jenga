#include "ef_rotation_control.h"

pid_params_t ef_pid_params = {
	.kp = 100.0,
	.ki = 0.0,
	.kd = 100.0,
	.min_output = 1500,
	.max_output = 5000,
};

pid_state_t ef_pid_state;

int16_t ef_angle = 0; //reference ef_angle
int16_t ef_set_point = 0; //where to go
int8_t ef_dir = 0;

int16_t get_ef_angle() {
  return ef_angle;
}

int16_t get_ef_set_point() {
  return ef_set_point;
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
  static uint16_t ef_duty = 0;

  // TODO: Change timer interval to avoid this counter
  if (count == 0) {
    if (ef_set_point > ef_angle) {
      set_pid_direction(&ef_pid_state, PID_DIRECT);
      ef_ccw();
    } else if (ef_set_point < ef_angle) {
      set_pid_direction(&ef_pid_state, PID_REVERSE);
      ef_cw();
    } else {
      ef_halt();
    }

    ef_duty = step_pid(&ef_pid_state, ef_set_point, ef_angle);
  }
  count++;

  return ef_duty;
}

void ef_rotation_encoder() {
  if (bit_is_set(PINF,SE2_PIN)) {
    ef_dir = -1;
    ef_angle++;
  } else {
    ef_dir = 1;
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
