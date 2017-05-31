#ifndef _COMMANDS_H_
#define _COMMANDS_H_

#include <avr/io.h>
#include <util/delay.h>
#include "uart_functions.h"
#include "arm_control.h"
#include "arm_paths.h"
#include "zaxis_control.h"
#include "ef_rotation_control.h"

void wait_for_command();

void handle_heartbeat_command();
void handle_arm_movement_command();
void handle_zaxis_home_command();
void handle_fire_solenoid_command();

#endif
