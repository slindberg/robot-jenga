#ifndef EF_ROTATION_CONTROL_H
#define EF_ROTATION_CONTROL_H

#include <avr/io.h>
#include "def.h"
#include "pid.h"

//about ~475 ticks per rev

int16_t get_ef_angle();
int16_t get_ef_set_point();

void init_ef_pid();

void rotate_ef(int8_t delta);
void rotate_ef_absolute(int16_t set_point);

void ef_rotation_encoder();
uint16_t process_ef_rotation();

void ef_cw();
void ef_ccw();
void ef_halt();

#endif
