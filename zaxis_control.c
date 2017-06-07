#include "zaxis_control.h"

pid_params_t z_pid_params = {
	.kp = 0.25,
	.ki = 0.05,
	.kd = 0.0,
	.min_output = 4200,
	.max_output = 5000,
};

pid_state_t z_pid_state;

int32_t z_set_point = 0; //where to go
int32_t z_position = 0; //reference position
int16_t z_duty = 0; //pwm duty cycle
int8_t z_dir = 0; //1 down, -1 is up, 0 is undefined
uint8_t run_zmotor = 0; //enables the motor to run or nah
uint8_t z_homing = 0; //whether the z-axis is currently homing
//break_dir keeps track of which limit switch it hit
int8_t z_break_dir = 0; //1 down, -1 up, 0 undefined

int32_t get_zaxis_position() {
  return z_position;
}

int32_t get_zaxis_set_point() {
  return z_set_point;
}

uint16_t get_zaxis_duty() {
  return z_duty;
}

pid_params_t *get_zaxis_pid_params() {
  return &z_pid_params;
}

void init_zaxis_pid() {
  init_pid_state(&z_pid_state, &z_pid_params);
}

void home_zaxis() {
  // run the zaxis down until it hits the limit switch
  z_homing = 1;
  move_zaxis_absolute(-INT32_MAX);
}

void move_zaxis(int32_t delta) {
  move_zaxis_absolute(z_position + delta);
}

void move_zaxis_absolute(int32_t set_point) {
  if ((z_break_dir != 1 || z_position >= set_point) &&
      (z_break_dir != -1 || z_position <= set_point)) {
    z_set_point = set_point;
    run_zmotor = 1;
    z_break_dir = 0;
  }
}

//gets called with timer0
uint16_t process_zaxis() {
  static uint8_t count = 0;

  if (run_zmotor) {
    if (z_set_point == z_position) {
      z_halt();
      z_duty = 0;
      reset_pid_error(&z_pid_state);
    } else {
      if (z_set_point > z_position) {
        set_pid_direction(&z_pid_state, PID_DIRECT);
        z_up();
      } else {
        set_pid_direction(&z_pid_state, PID_REVERSE);
        z_down();
      }
    }

    if (count == 0) {
      z_duty = step_pid(&z_pid_state, z_position, z_set_point);
    }
  }

  count++;

  return z_duty;
}

void zaxis_limit_switch() {
  if (z_set_point > z_position) {
    z_break_dir = 1;
  } else {
    z_break_dir = -1;

    if (z_homing) {
      z_position = 0;
      z_homing = 0;
    }
  }
  z_set_point = z_position;
}

void zaxis_encoder() {
  if (bit_is_set(PINB, ZE2_PIN)) {
    z_dir = -1; //down
    z_position--;
  } else {
    z_dir = 1; //up
    z_position++;
  }
}

void z_down() {
  PORTB |= (1<<ZMOTOR_R); //PB0
  PORTB &= ~(1<<ZMOTOR_L); //PB1
}

void z_up() {
  PORTB &= ~(1<<ZMOTOR_R); //PB0
  PORTB |= (1<<ZMOTOR_L);  //PB1
}

void z_halt() {
  PORTB &= ~((1<<ZMOTOR_R) | (1<<ZMOTOR_L));
  run_zmotor = 0;
}
