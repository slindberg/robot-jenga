/* Ernie Bodle - 2017 - 05 - 11
 * robot.c 
 * This lab uses a modified version of my lab4 ECE473 code.
 * 
 * This is for my ROB 421 project
 * Jenga Robot 
 */ 

/*HAVE THE FIRST ITERATION DO:
 * Make the arm/catcher go out to a position and then retract
 * Make it shoot
 * And reel everything up
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <math.h>
#include <stdlib.h>
#include "uart_functions.h"
#include "motor_functions.h"
#include "def.h" //functions with my DEFINES in it

int8_t step_intervals_left[] = {
5,7,7,10,12,14,18,21,25,28,32,36,39,42,44,48,49,51,52,51,52,52,51,52,51,51,51,51,50,51,50,50,50,50,50,50,49,50,49,49,50,50,50,50,49,50,50,50,50,49,49,50,49,48,49,49,48,48,48,48,47,45,43,41,38,35,32,29,26,22,20,16,13,11,9,7,5,5
};

int8_t step_intervals_right[] = {
1,1,2,2,3,3,5,5,7,8,9,11,13,14,15,17,19,19,20,21,21,22,22,23,23,24,24,24,25,25,26,26,26,27,28,28,28,29,30,30,34,38,41,43,46,48,50,53,55,57,59,61,62,65,65,68,68,70,72,72,72,70,69,66,62,58,53,48,44,38,32,28,23,18,15,12,10,8
};

//--------------------------
//ADC
uint16_t adc_num = 0; //get rid of this
uint16_t IR0_num = 0; 	//this is the number outputted by the adc
uint16_t IR1_num = 0; 	//this is the number outputted by the adc
uint16_t IR2_num = 0; 	//this is the number outputted by the adc
void adc_init();
ISR(ADC_vect);
//--------------------------


//--------------------------
//solinoid and z motor
int16_t sol_position = 0; //reference sol_position
int16_t sol_des_pos = 0; //where to go
int8_t sol_dir = 0;
int8_t s_temp = 0;

int16_t z_position = 0; //reference z_position
int16_t z_des_pos = 0; //where to go
int8_t z_dir = 0;
int8_t z_temp = 0;


//ERNIE make sure these actually go CW and CCW
void z_cw();
void z_ccw();
void z_halt();

void solinoid_cw();
void solinoid_ccw();
void solinoid_halt();

void tcnt0_init(void);
ISR(TIMER0_OVF_vect);
//--------------------------

//sensor interrupts
ISR(INT0_vect);
ISR(INT1_vect);
ISR(INT2_vect);

void rob_init();

//--------------------------
//stepper controls
int16_t deltaL = 0; 
int16_t deltaR = 0; 

//index stuff
uint16_t right_index = 0; //right 
uint16_t left_index = 0; //left


void tcnt1_init();
ISR(TIMER1_COMPA_vect);
ISR(TIMER1_COMPB_vect);
//there is more in robot_functions.h
//--------------------------

//for pwm
void tcnt2_init(); //solinoid motor
void tcnt3_init(); //for catcher motor

void get_IR_data();

void get_array(){
    uint8_t i = 0;
    uint8_t temp = 0;
    char c = '0';
    char str[3];

    for(i = 0; i < 3; i++){
	c = uart_getc();
	str[i] = c;
    }

    temp = atoi(str);
    temp++;	

    uart_puts(itoa(temp, "", 10));
}

int main(){

    //setting up pins
    rob_init();

    //-------------------
    //interrupts
    tcnt0_init(); //motor control
    tcnt1_init(); //steppers
    tcnt2_init(); //pwm for solinoid motor
    tcnt3_init(); //pwm for catcher

    adc_init();
    uart_init(); //uart
    sei(); //enable all interrupts before entering loop
    //-------------------

    //start of jenga protocol

    //check for orthagonality here

    //make it go down all the way until it hits the bot/top z sensor


    while(1){

	//-----------------------------------------
	//debug
	

    //uart_puts(itoa(sol_position, "", 10));
    
    //z_des_pos += 10;
    uart_puts("Before\n");
    uart_puts(itoa(123, "", 10));
    uart_puts("\n");
   _delay_ms(100); 
    
   //z_des_pos -= 10;
    
   uart_puts("After\n");
    uart_puts(itoa(z_position, "", 10));
    uart_puts("\n");
   _delay_ms(100); 
	
   //get_array();

	/*
	   sol_des_pos += 475/4; 
	   deltaL += 800 * (96/20);
	   deltaR += 800 * (72/20);
	   _delay_ms(5000);  //why the hell is this 5 seconds?

	   PORTF |= (1<<S_TRIG);	
	   _delay_ms(20);  //why the hell is this 5 seconds?
	   PORTF &= ~(1<<S_TRIG);	

	sol_des_pos -= 475/4; 
	_delay_ms(5000); 
	*/
	//end debug	
	//-----------------------------------------

	//1. wait for on signal
	//i = 0;
	//while(i == 0){
	//if(check pin here){
	//
	//} //end if
	//} //end while


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


    } //end while

    return 0; //do I need this in a uC?
} //end main

void rob_init(){

    //A
    DDRA |=  (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R);
    //PORTA will be 0 initally

    //B
    DDRB |= (1<<ZMOTOR_R) | (1<<ZMOTOR_L) | 
	(0<<ZE1) | (0<<ZE2) | (1<<CATCHER) | (1<<7);
    //^ 7 is pwm    
    DDRB |= (1<<0) | (1<<1); //debug
    PORTB |= (1<<ZE1) | (1<<ZE2);

    //D
    //DDRD |= (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R)
    DDRD |= (0<<TOP_Z) | (0<<BOT_Z) | (0<<CATCHER_SENSE);
    PORTD |= (1<<TOP_Z) | (1<<BOT_Z) | (1<<CATCHER_SENSE);

    //E
    DDRE |= (0<<TRANS) | (1<<RECEV) | 
	(0<<START_SIG) | (1<<END_SIG) | (0<<ORTHO);
    DDRE |= (1<<3); //pwm
    PORTE |= (1<<TRANS) | (1<<ORTHO) | (1<<START_SIG);

    //F
    DDRF |= (0<<IR0) | (0<<IR1) | (0<<IR2)
	| (1<<S_TRIG) | (1<<SR) | (1<<SL) | (0<<SE1) | (0<<SE2);
    PORTF |= (0<<IR0) | (0<<IR1) | (0<<IR2) | (1<<SE1) | (1<<SE2);


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
    //clk/256
    TCCR1A = 0x00; 
    TCCR1B |= (1<<CS11) | (1<<CS10); //clk/64
    TCCR1C = 0x00;

    //these don't actually change at differnt rates...
    //right stepper
    TIMSK |= (1<<OCIE1A); //output compair match
    OCR1A |= 0x40; //64 

    //left stepper
    //TIMSK |= (1<<OCIE1B); //output compair match
    //OCR1B |= 0xB1; 

} //end tcnt1 init

#define ACCEL_RATE 5 //test it being one for now!
#define MAX_DAMP 0x40
#define MIN_DAMP 0x01

ISR(TIMER1_COMPA_vect){ 
    TCNT1 = 0; //change this to CTC mode later
    //^ CTC mode was giving me interrupt and pin errors

    //clears the steps so it can pulse
    PORTA &= ~((1<<STEP_L) | (1<<STEP_R));

    static StepperState left_stepper = {
      .travel_mask = 1 << STEP_L,
      .direction_mask = 1 << DIR_L,
      .step_intervals = step_intervals_left,
      .total_intervals = sizeof(step_intervals_left),
      .interval_index = 0,
      .steps_remaining = 0,
      .delay_count = 0,
      .interval_count = 0
    };

    static StepperState right_stepper = {
      .travel_mask = 1 << STEP_R,
      .direction_mask = 1 << DIR_R,
      .step_intervals = step_intervals_right,
      .total_intervals = sizeof(step_intervals_right),
      .interval_index = 0,
      .steps_remaining = 0,
      .delay_count = 0,
      .interval_count = 0
    };

    stepper(&left_stepper);
    stepper(&right_stepper);
}

ISR(TIMER1_COMPB_vect){
    //do nothing for now
} //comp timer 1
//end stepper controls
//-----------------------------------------------------


//-----------------------------------------------------
//pwm
void tcnt2_init(void){
    //enable timer/counter compare interrupt2 
    TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<COM21) | 
	(1<<COM20) | (1<<CS21) | (1<<CS20);
    //TIMSK |= (1<<TOIE2);
    OCR2 = 185;
}
//-----------------------------------------------------



//---------------------------------------------------------------------
//this is motor stuff
void tcnt0_init(void){
    //DDRB |= 0x01;
    TCCR0 |= (1<<CS00);
    ASSR |= (0<<AS0);
    TIMSK |= (1<<TOIE0);
    //OCR0 = 0x0F; 	//this gets compaired with TCNT0 counter

    //ASSR   |=  (1<<AS0);  	//using the external clock
    //TIMSK  |=  (1<<TOIE0); //(1<<OCIE0); 	//enable timer/counter0 compare interrupt
    //TCCR0  |= (1<<CS02) | (1<<CS01); 	//256 mode
    //OCR0 = 0x0F; 	//this gets compaired with TCNT0 counter
} //end tcnt)_init

//ERNIE change this to compare vector?
ISR(TIMER0_OVF_vect){
    //ERNIE
    //Have this drive both motors at once!!!

    //make this into a function

    //-----------------------------------------------
    //Solinoid encoder
    if(s_temp == 0){
	if( (bit_is_clear(PINF,SE1_PIN) || bit_is_clear(PINF,SE2_PIN)) ){
	    s_temp = 1;
	    //lets se how much of a delay we need
	    _delay_us(100); //micro seconds, so not too bad?

	    if(bit_is_clear(PINF,SE1_PIN)){
		sol_dir = 1;
		sol_position++;
	    }

	    if(bit_is_clear(PINF,SE2_PIN)){
		sol_dir = -1;
		sol_position--;
	    }
	}	    
    }
    if((bit_is_set(PINF,SE1_PIN)) && bit_is_set(PINF,SE2_PIN)){
	s_temp = 0;
	sol_dir = 0;
    }
    //-----------------------------------------------

    //-----------------------------------------------
    //Solinoid encoder
    if(z_temp == 0){
	if( (bit_is_clear(PINB,ZE1_PIN) || bit_is_clear(PINB,ZE2_PIN)) ){
	    z_temp = 1;
	    //lets se how much of a delay we need
	    _delay_us(100); //micro seconds, so not too bad?

	    if(bit_is_clear(PINB,ZE1_PIN)){
		z_dir = 1;
		z_position++;
	    }

	    if(bit_is_clear(PINB,ZE2_PIN)){
		z_dir = -1;
		z_position--;
	    }
	}	    
    }
    if((bit_is_set(PINB,ZE1_PIN)) && bit_is_set(PINB,ZE2_PIN)){
	z_temp = 0;
	z_dir = 0;
    }
    //-----------------------------------------------


    //ERNIE Get PID contorl to work!!!
    //zmotor
    if(z_des_pos == z_position){
	z_halt();   
    } 
    if(z_des_pos > z_position){
	z_cw();
    }
    if(z_des_pos < z_position){
	z_ccw();
    }

    //solinoid motor
    if(sol_des_pos == sol_position){
	solinoid_halt();   
    } 
    if(sol_des_pos > sol_position){
	solinoid_cw();
    }
    if(sol_des_pos < sol_position){
	solinoid_ccw();
    }

} //end timer0 ISR

void z_cw(){
    PORTB &= ~(1<<ZMOTOR_L);
    PORTB |= (1<<ZMOTOR_R);
}

void z_ccw(){
    PORTB &= ~(1<<ZMOTOR_R);
    PORTB |= (1<<ZMOTOR_L);
}

void z_halt(){
    PORTB &= ~((1<<ZMOTOR_R) | (1<<ZMOTOR_L));
}

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

//end motor stuff
//---------------------------------------------------------------------

//--------------------------------------------
//Catcher motor pwm
void tcnt3_init(){
    //Fast PWM, TOP: ICR3, update: BOT, TOV3 set on TOP
    //clear when (OCR3A == TCNT3), set on compare match 
    TCCR3A |= (1<<WGM31) | (1<<COM3A1);
    TCCR3B |= (1<<WGM33) | (1<<WGM32); 

    //1/64 scaler
    TCCR3B |=  (1<<CS31) | (1<<CS30);  

    //Do I even need this?
    ETIMSK |= (1<<OCIE3A);

    //For scaling 
    ICR3 = 5000; //Top
    OCR3A = 250; //spool
    //OCR3A = 375; //stop
    //OCR3A = 250; //other
} //tcnt3_init
//--------------------------------------------


//--------------------------------------------
//adc stuff
//ERNIE fix this shit so IRs work!!!
void adc_init(){
    DDRF |= (0<<IR0) | (0<<IR1) | (0<<IR2);
    ADMUX |= (1<<REFS0); //using AVcc with external cap

    ADMUX &= ~ ((1<<MUX0) | (1<<MUX1)); //clearing all bits to set ADC0
    //ADMUX |= (1<<MUX0); //after clearing this will enable ADC1
    //ADMUX |= (1<<MUX1); //after clearing this will enable ADC2

    //enables adc and interrupt
    ADCSRA |= (1<<ADEN) | (1<<ADIE); 
    //scales it down by 128 (idk what clock it uses)
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
} //end adc init

ISR(ADC_vect){
    adc_num = ADC;
    //why do I do this again?
    ADCSRA |= (1<<ADSC);

    if(bit_is_clear(ADMUX, MUX0) && bit_is_clear(ADMUX, MUX1)){
	IR0_num = adc_num;
    } else if(bit_is_set(ADMUX, MUX0)) {
	IR1_num = adc_num;
    } else {
	IR2_num = adc_num;
    }

    /*debug
      if(adc_num > 300)
      PORTB |= (1<<0);
      else 
      PORTB &= ~(1<<0);

      if(adc_num > 400){
      PORTB |= (1<<1);
      } else {
      PORTB &= ~(1<<1);
      }*/
} //ISR ADC
//--------------------------------------------


void get_IR_data(){
    ADMUX &= ~ ((1<<MUX0) | (1<<MUX1)); //clearing all bits to set ADC0
    ADCSRA |= (1<<ADSC); //starts the ADC conversions
    uart_puts("IR0: ");
    uart_puts(itoa(IR0_num, "Hello", 10));
    uart_putc('\n');
    uart_putc(((char) 13));
    //_delay_ms(50);	

    ADMUX |= (1<<MUX0); //after clearing this will enable ADC1
    ADCSRA |= (1<<ADSC); //starts the ADC conversions
    uart_puts("IR1: ");
    uart_puts(itoa(IR1_num, "Hello", 10));
    uart_putc('\n');
    uart_putc(((char) 13));
    //_delay_ms(50);	

    ADMUX &= ~ ((1<<MUX0) | (1<<MUX1)); //clearing all bits to set ADC0
    ADMUX |= (1<<MUX1); //after clearing this will enable ADC1
    ADCSRA |= (1<<ADSC); //starts the ADC conversions
    uart_puts("IR2: ");
    uart_puts(itoa(IR2_num, "Hello", 10));
    uart_putc('\n');
    uart_putc(((char) 13));
    //_delay_ms(50);	
}


