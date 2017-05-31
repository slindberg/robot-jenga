#include "commands.h"

// Preallocated array buffers for custom path intervals
int8_t left_intervals[256];
int8_t right_intervals[256];

arm_path_t custom_arm_path = {
  .left = {
    .intervals = left_intervals,
    .length = 0,
  },
  .right = {
    .intervals = right_intervals,
    .length = 0,
  }
};

void wait_for_command() {
  char command = read_command();

  switch (command) {
    case '.':
      // heartbeat
      break;

    case 'H':
      handle_zaxis_home_command();
      break;

    case 'Z':
      handle_zaxis_move_command();
      break;

    case 'P':
      handle_predefined_arm_move_command();
      break;

    case 'A':
      handle_custom_arm_move_command();
      break;

    case 'R':
      handle_ef_rotate_command();
      break;

    case 'F':
      handle_fire_solenoid_command();
      break;

    default:
      // bad command
      write_str("X");
      return;
  }

  write_str(".");
}

void handle_zaxis_home_command() {
  home_zaxis();
}

void handle_zaxis_move_command() {
  // The next four bytes are the signed number of encoder steps to take
  int16_t distance = read_int16();

  move_zaxis(distance);
}

void handle_predefined_arm_move_command() {
  // The next two bytes are the number of the predefined path
  int path_number = read_int8();

  start_arm_path(&arm_paths[path_number - 1]);
}

void handle_custom_arm_move_command() {
  // The next two bytes are the number of intervals in the path
  char path_size = read_int8();

  // The remaining bytes represent the paths, left then right
  read_int8_array(left_intervals, path_size);
  read_int8_array(right_intervals, path_size);

  custom_arm_path.left.length = path_size;
  custom_arm_path.right.length = path_size;

  start_arm_path(&custom_arm_path);
}

void handle_ef_rotate_command() {
  // The next four bytes are the signed number of encoder steps to take
  int16_t angle = read_int16();

  rotate_ef(angle);
}

void handle_fire_solenoid_command() {
  PORTF |= (1<<S_TRIG);
  _delay_ms(20);
  PORTF &= ~(1<<S_TRIG);
}

