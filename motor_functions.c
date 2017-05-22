//2017 - 05 - 16

#include "motor_functions.h"
#define STEP_PORT PORTA


void stepper(uint16_t *count, int16_t *prev_position, int16_t *position, uint8_t step, uint8_t dir, uint16_t damp, int16_t delta){ 
    //Left
    if(delta != (*position)){
	if((*count) == damp){
	    (*count) = 0;
	    //STEP_PORT ^= (1<<step);
	    

	    if(delta > (*position)){
		//STEP_PORT ^= (1<<step); 
		STEP_PORT |= (1<<step); 
		
		
		STEP_PORT |= (1<<dir); 
		(*position)++;
	    } //if delta > position
	    if(delta < (*position)){
		//STEP_PORT ^= (1<<step);
		STEP_PORT |= (1<<step); 
		STEP_PORT &= ~(1<<dir);
		(*position)--;
	    } //if delta < position
	} else {
	    (*count)++;
	} //count == damp
    } else {
	(*prev_position) = delta;
    }
}
//end stepper controls
//--------------------------------------------------


