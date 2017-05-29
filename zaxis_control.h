#ifndef ZAXIS_H
#define ZAXIS_H

#include <avr/io.h>
#include "def.h"
#include "pid.h"

//about 100 revs * 1000 counts

void move_zaxis(int32_t des_pos);
void move_zaxis_absolute(int32_t des_pos);

void z_up();
void z_down();
void z_halt();
void zaxis_limit_switch();
void zaxis_encoder();
uint16_t process_zaxis();
void home_zaxis();

#endif
