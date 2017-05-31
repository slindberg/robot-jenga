#include "commands.h"

void wait_for_command() {
  char command = uart_getc();

  switch (command) {
    // heartbeat
    case 'P':
    case 'p':
      handle_heartbeat_command();
      break;

      // arm movement
    case 'A':
    case 'a':
      handle_arm_movement_command();
      uart_puts("Done");
      break;

      // rotate end effector
    case 'R':
    case 'r':
      uart_putc('R');
      break;

    case 'H':
    case 'h':
      handle_zaxis_home_command();
      break;

      // z-axis down one block
    case 'Z':
    case 'z':
      move_zaxis(-3675);
      uart_puts("Done");
      break;

      // z-axis up one block
    case 'X':
    case 'x':
      move_zaxis(3675);
      uart_puts("Done");
      break;

    case 'W':
    case 'w':
      rotate_ef(-50);
      uart_puts("Done");
      break;

    case 'C':
    case 'c':
      rotate_ef(50);
      uart_puts("Done");
      break;

      // fire solenoid
    case 'F':
    case 'f':
      handle_fire_solenoid_command();
      break;
  }
}

void handle_heartbeat_command() {
  uart_putc('.');
}

void handle_arm_movement_command() {
  char path_number = uart_getc();
  uint8_t path_index = (path_number - '0') - 1;

  uint8_t result = start_arm_path(&arm_paths[path_index]);

  if (!result) {
    uart_puts("Fail");
    return;
  }

  // Wait on arm movement to complete
  while (!is_arm_path_complete());
  uart_puts("Done");
}

void handle_zaxis_home_command() {
  // Just use a
  home_zaxis();
  uart_puts("Done");
}

void handle_fire_solenoid_command() {
  PORTF |= (1<<S_TRIG);
  _delay_ms(20);
  PORTF &= ~(1<<S_TRIG);
  uart_puts("Done");
}
