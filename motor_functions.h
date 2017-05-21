//2017 - 05 - 16

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>

void tcnt2_init(void);

//--------------------------
//stepper controls
//these need to be global in the driver functions
//int16_t deltaL = 0; 
//int16_t deltaR = 0; 

//ERNIE should I pass in a struct? Or does it not matter?
void stepper(uint16_t *count, int16_t *prev_position, int16_t *position, 
	uint8_t step, uint8_t dir, uint16_t damp, int16_t delta); 
//end stepper controls
//--------------------------

