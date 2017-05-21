/* Ernie Bodle - 2017 - 05 - 11
 * motor.c 
 * This lab uses a modified version of my lab4 ECE473 code.
 * 
 * This is for my ROB 421 project
 * For the stepper motor micro stepper chip
 * Jenga Robot 
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>
#include <stdlib.h>
#include "uart_functions.h"
#include "motor_functions.h"
#include "def.h"


//sensor interrupts
ISR(INT0_vect);
ISR(INT1_vect);
ISR(INT2_vect);

void rob_init();

//HAVE THE FIRST ITERATION DO:
//Make the arm/catcher go out to a position and then retract
//Make it shoot
//And reel everything up



//--------------------------
//stepper controls
int16_t deltaL = 0; 
int16_t deltaR = 0; 

void tcnt1_init();
ISR(TIMER1_COMPA_vect);
//there is more in robot_functions.h
//--------------------------

int main(){

    //variables
    //int i;

    //setting up pins
    rob_init();

    //-------------------
    //interrupts
    tcnt1_init(); //steppers


    
    //tcnt2 is messing with the steppers
    //it's probably the xck1 pin
    //tcnt2_init(); //pwm for catcher servo
    
    
    sei(); //enable interrupts before entering loop



    //check for orthagonality here
    
    //make it go down all the way until it hits the bot/top z sensor

    while(1){
	//1. wait for on signal
	//i = 0;
	//while(i == 0){
	//if(check pin here){
	//
	//} //end if
	//} //end while
	

	//deltaL += 800;
	//deltaR += 800;
	_delay_ms(500); //wait 5 sec
	deltaL -= 800;
	deltaR -= 800;
	_delay_ms(500); //wait 5 sec

	//2a. let catcher down
	
	//pwm
	//make the cater motor loose
	//OCR0 = 0x00?
		
	//2b. extend arm out
	
	//figure out where to go
	//deltaR = ?;
	//deltaL = ?;
	//deltaZ = ?;
	
	//while(invalid block){
	//3. move to the desired row and position
	
	//4. Analyze if the robot has a valid block
	
	//5. If not then go back to 3
	//} //end while invalid block
	
	
	
	//6. position servo so it's flush
	
	//7. shoot! and save the data that it hit the block
	
	//PORTF |= (1<<S_TRIG); //on
	//_delay_ms(1ms);
	//PORTF &= ~(1<<S_TRIG); //off

	//8. reel everything back up
	//turn pwm back on
	//
	
	//9. send done signal
	//PORT
    
	_delay_ms(100); //get rid of this
    }

    return 0; //do I need this in a uC?
}

void rob_init(){
    
    //B
    DDRB |= (1<<ZMOTOR_R) | (1<<ZMOTOR_L) | 
	(0<<ZE1) | (0<<ZE2) | (1<<CATCHER);    
    PORTB |= (1<<ZE1) | (1<<ZE2);

    //D
    DDRD |= (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R)
	| (0<<TOP_Z) | (0<<BOT_Z) | (0<<CATCHER_SENSE);
    PORTD |= (1<<TOP_Z) | (1<<BOT_Z) | (1<<CATCHER_SENSE);

    //E
    DDRE |= (0<<TRANS) | (1<<RECEV) | 
	(0<<START_SIG) | (1<<END_SIG) | (0<<ORTHO);
    PORTE |= (1<<TRANS) | (1<<ORTHO) | (1<<START_SIG);

    //F
    DDRF |= (0<<IR0) | (0<<IR1) | (0<<IR2)
	| (1<<S_TRIG) | (1<<SR) | (1<<SL) | (0<<SE1) | (0<<SE2);
    PORTF |= (1<<IR0) | (1<<IR1) | (1<<IR2) | (1<<SE1) | (1<<SE2);


    //enable interrupts for INTx rising edge
    EICRA &= ~ ( (1<<ISC01) | (1<<ISC00) );
    EICRA &= ~ ( (1<<ISC11) | (1<<ISC10) );
    EICRA &= ~ ( (1<<ISC21) | (1<<ISC20) );
    
    //enable interrupts
    EIMSK |= (1<<INT0) | (1<<INT1) | (1<<INT2);
}


ISR(INT0_vect){
    //PD0
    //top Z sensor
    //have this stop the Z sensor

}

ISR(INT1_vect){
    //PD1
    //bot Z sensor
    //have this
}


ISR(INT2_vect){

    //PD2
    //have this stop the PWM for the catcher
}


//--------------------------------------------------
//stepper controls
void tcnt1_init(void){
    TCCR1A |= 0x00; 
    TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);
    TIMSK |= (1<<OCIE1B);

    OCR1B |= 0x01;
} //end tcnt1 init

#define ACCELL_RATE 1 //test it being one for now!
#define MAX_DAMP 128
#define MIN_DAMP 24

ISR(TIMER1_COMPB_vect){
    TCNT1 = 0; //change this to CTC mode later
    static uint16_t countL = 0;
    static uint16_t countR = 0;

    //this was for quarter step
    //24 is the lowest (fastest) number
    //64 is still pretty fast
    //76 still fast but getting a little jumpy
    //92 
    //128 is jumpy
    //255 does not work so somewhere from 128 to 255 it breaks


    //deltaR/deltaL are where we want to go
    //where we are 
    static int16_t positionL = 0;
    static int16_t positionR = 0;

    //this is keeping track of our inital position
    static int16_t prev_positionL = 0;
    static int16_t prev_positionR = 0;

    //basically the inverse of velocity
    //this makes the triangle
    //static int16_t dampL; 
    //static int16_t dampR; 

    //this is the actual inverse of the speed used
    //this makes the trapazoid 
    static uint16_t used_dampL = 76;  
    static uint16_t used_dampR = 76;

    //midpoint of the triangle
    //int16_t midL = deltaL - prev_positionL;
    //int16_t midR = deltaR - prev_positionR;

    //ERNIE get trapazoid working
    //ERNIE make it so both A and B flow at the same rate
    stepper(&countL, &prev_positionL, &positionL, STEP_L, DIR_L, used_dampL, deltaL); 
    stepper(&countR, &prev_positionR, &positionR, STEP_R, DIR_R, used_dampR, deltaR); 

} //comp timer 1
//end stepper controls
//-----------------------------------------------------





