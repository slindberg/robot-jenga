//2017 - 05 - 16

#include "motor_functions.h"


//pwm
void tcnt2_init(void){
    //enable timer/counter compare interrupt2 
    TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<COM21) | 
	(1<<COM20) | (1<<CS21) | (1<<CS20);
    TIMSK |= (1<<TOIE2);
    OCR2 = 0x0F;
}


void stepper(uint16_t *count, int16_t *prev_position, int16_t *position, uint8_t step, uint8_t dir, uint16_t damp, int16_t delta){ 
    //Left
    if(delta != (*position)){
	if((*count) == damp){
	    (*count) = 0;
	    PORTD ^= (1<<step);

	    if(delta > (*position)){
		PORTD ^= (1<<step); 
		PORTD |= (1<<dir); 
		(*position)++;
	    } //if delta > position
	    if(delta < (*position)){
		PORTD ^= (1<<step);
		PORTD &= ~(1<<dir);
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


