#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart_functions.h"

char read_command();

void read_chars(char *buffer, uint8_t length);
int8_t read_int8();
int16_t read_int16();
void read_int8_array(int8_t *intervals, uint8_t length);

void write_int(int32_t value);
void write_str(char *response);


#endif
