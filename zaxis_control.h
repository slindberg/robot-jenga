#ifndef ZAXIS_CONTROL_H
#define ZAXIS_CONTROL_H

#include <avr/io.h>
#include "def.h"
#include "pid.h"

//about 100 revs * 1000 counts

int32_t *get_zaxis_position();
int32_t *get_zaxis_set_point();
pid_params_t *get_zaxis_pid_params();

void init_zaxis_pid();

void home_zaxis();
void move_zaxis(int32_t delta);
void move_zaxis_absolute(int32_t set_point);

void z_up();
void z_down();
void z_halt();

void zaxis_limit_switch();
void zaxis_encoder();
uint16_t process_zaxis();

#endif
