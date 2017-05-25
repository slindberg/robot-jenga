#ifndef _ARM_PATHS_H_
#define _ARM_PATHS_H_

#include <avr/io.h>
#include "motor_functions.h"

typedef struct ArmPath {
  StepperMovement left;
  StepperMovement right;
} ArmPath;

ArmPath arm_paths[2];

#endif // _ARM_PATHS_H_
