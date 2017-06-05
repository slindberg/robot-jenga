#include "commands.h"

// Preallocated array buffers for custom path intervals
int8_t left_intervals[256];
int8_t right_intervals[256];

arm_path_t arm_path = {
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
  char *response = "."; // default response

  // there's a timeout in uart_getc that returns 0, don't treat it as a command
  if (command == 0) {
    return;
  }

  start_response();

  switch (command) {
    case '.':
      // heartbeat
      break;

    case 'T':
      response = handle_turn_check_command();
      break;

    case 'B':
      handle_begin_turn_command();
      break;

    case 'E':
      response = handle_end_turn_command();
      break;

    case 'H':
      handle_zaxis_home_command();
      break;

    case 'Z':
      handle_zaxis_move_command();
      break;

    case 'A':
      handle_arm_move_command();
      break;

    case 'R':
      handle_ef_rotate_command();
      break;

    case 'F':
      handle_fire_solenoid_command();
      break;

    default:
      handle_bad_command(command);
      return;
  }

  write_str(response);
  end_response();
}

char *handle_turn_check_command() {
  if (bit_is_set(PINA, START_SIG_PIN)) {
    return "1";
  } else {
    return "0";
  }
}

void handle_begin_turn_command() {
  lower_catcher();
}

char *handle_end_turn_command() {
  raise_catcher();
  return "!";
}

void handle_zaxis_home_command() {
  home_zaxis();
}

void handle_zaxis_move_command() {
  // The next four bytes are the signed number of encoder steps to take
  int16_t distance = read_int16();

  move_zaxis(distance);

  // debug_zaxis_position();
}

void handle_arm_move_command() {
  // The next two bytes are the number of intervals in the path
  char path_size = read_int8();

  // The remaining bytes represent the paths, left then right
  read_int8_array(left_intervals, path_size);
  read_int8_array(right_intervals, path_size);

  arm_path.left.length = path_size;
  arm_path.right.length = path_size;

  start_arm_path(&arm_path);
}

void handle_ef_rotate_command() {
  // The next four bytes are the signed number of encoder steps to take
  int16_t angle = read_int16();

  rotate_ef(angle);

  // debug_ef_position();
}

void handle_fire_solenoid_command() {
  PORTF |= (1<<S_TRIG);
  _delay_ms(20);
  PORTF &= ~(1<<S_TRIG);
}

void handle_bad_command(char command) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "Bad command: '%c'", command);
  write_str(buffer);
}

void debug_ef_position() {
  int16_t current_angle, set_point;
  char buffer[32];

  do {
    current_angle = get_ef_angle();
    set_point = get_ef_set_point();
    snprintf(buffer, sizeof(buffer), "%d %d %d\n", current_angle, set_point, OCR1C);
    write_str(buffer);
    // _delay_ms(10);
  } while (current_angle != set_point);
}

void debug_zaxis_position() {
  int16_t current_angle, set_point;
  char buffer[32];

  do {
    current_pos = get_zaxis_position();
    set_point = get_zaxis_set_point();
    snprintf(buffer, sizeof(buffer), "%d %d %d\n", current_pos, set_point, OCR1B);
    write_str(buffer);
    // _delay_ms(10);
  } while (current_angle != set_point);
}

