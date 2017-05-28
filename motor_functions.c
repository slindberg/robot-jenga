//2017 - 05 - 16

#include "motor_functions.h"

//ERNIE make sure these are actually cw
void solinoid_cw(){
    PORTF &= ~(1<<SL);
    PORTF |= (1<<SR);
}

void solinoid_ccw(){
    PORTF &= ~(1<<SR);
    PORTF |= (1<<SL);
}

void solinoid_halt(){
    PORTF &= ~((1<<SR) | (1<<SL));
}
