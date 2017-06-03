#include "protocol.h"

char read_command() {
  char command = uart_getc();

  // capitalize letter commands
  if (command >= 97 && command <= 122) {
    command -= 32;
  }

  return command;
}

void read_chars(char *buffer, uint8_t length) {
  for (int i = 0; i < length; i++)
    buffer[i] = uart_getc();
}

int8_t read_int8() {
  char hex_str[2];
  read_chars(hex_str, 2);
  return (int8_t)strtol(hex_str, NULL, 16);
}

int16_t read_int16() {
  char hex_str[4];
  read_chars(hex_str, 4);
  return (int16_t)strtol(hex_str, NULL, 16);
}

void read_int8_array(int8_t *intervals, uint8_t length) {
  for (int i = 0; i < length; i++) {
    intervals[i] = read_int8();
  }
}

void write_int(int32_t value) {
  char buffer[20];
  write_str(itoa(value, buffer, 10));
}

void write_str(char *str) {
  uart_puts(str);
}

void start_response() {
  // Frame start character
  uart_putc(':');
}

void end_response() {
  // Frame end character
  uart_putc(';');
}
