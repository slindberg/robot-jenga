/* Ernie Bodle - 2017 - 05 - 11
 * robot.c 
 * This lab uses a modified version of my lab4 ECE473 code.
 * 
 * This is for my ROB 421 project
 * Jenga Robot 
 */ 

/* ERNIE
 * Should I move stepper motor stuff into tcnt2 and make tcnt1 
 * into pwm? Only do this if tcnt3 isn't very good.
 * (Or even make tcnt2 the servo pwm contorller and tcnt3 into 
 * my motor pwms)
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
#include "arm_control.h"
#include "arm_paths.h"
#include "zaxis_control.h"
#include "def.h" //functions with my DEFINES in it

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

//int16_t zduty = 0;

void tcnt0_init(void);
ISR(TIMER0_OVF_vect);
//--------------------------

//sensor interrupts
ISR(INT0_vect);
ISR(INT1_vect);
ISR(INT2_vect);

void rob_init();

//--------------------------

// arm stepper motors
void tcnt1_init();
ISR(TIMER1_COMPA_vect);

//for pwm
void tcnt2_init(); //servo pwm
void tcnt3_init(); //dc motors

void get_IR_data();

void handle_heartbeat_command();
void handle_arm_movement_command();
void handle_fire_solenoid_command();

void wait_for_command() {
  char command = uart_getc();

  switch (command) {
    // heartbeat
    case 'P':
    case 'p':
      handle_heartbeat_command();
      break;

      // arm movement
    case 'A':
    case 'a':
      handle_arm_movement_command();
      uart_puts("Done");
      break;

      // rotate end effector
    case 'R':
    case 'r':
      uart_putc('R');
      break;

      // z-axis movement
    case 'Z':
    case 'z':
      move_zaxis(-40000); 
      uart_puts(itoa(OCR3B, "",10));
      break;
    case 'X':
    case 'x':
      move_zaxis(40000); 
      uart_puts(itoa(OCR3B, "",10));
      break;

      // fire solenoid
    case 'F':
    case 'f':
      handle_fire_solenoid_command();
      break;
  }
}

void handle_heartbeat_command() {
  uart_putc('.');
}

void handle_arm_movement_command() {
  char path_number = uart_getc();
  uint8_t path_index = (path_number - '0') - 1;

  uint8_t result = start_arm_path(&arm_paths[path_index]);

  if (!result) {
    uart_puts("Fail");
    return;
  }

  // Wait on arm movement to complete
  while (!is_arm_path_complete());
  uart_puts("Done");
}

void handle_fire_solenoid_command() {
  PORTF |= (1<<S_TRIG);
  _delay_ms(20);
  PORTF &= ~(1<<S_TRIG);
  uart_puts("Done");
}

int main() {

  //figure out real min and max for pwm
  //max 5000; //Top of icr3
  //min 2500
  zpid_init();

  //setting up pins
  rob_init();

  //-------------------
  //interrupts
  tcnt0_init(); //DC motor control

  //steppers
  tcnt1_init(); //steppers
  init_stepper_pins(DIR_L, STEP_L, DIR_R, STEP_R);

  tcnt2_init(); //pwm for servo motor
  tcnt3_init(); //pwm for catcher and DC motors

  //adc_init();
  uart_init(); //uart
  sei(); //enable all interrupts before entering loop
  //-------------------

  //start of jenga protocol

  //check for orthagonality here

  //make it go down all the way until it hits the bot/top z sensor

  while(1){

    //-----------------------------------------
    //debug
    //uart_puts(itoa(zduty, "", 10));
    //uart_putc(10); //linefeed
    // uart_putc(13); //carrage return
    //_delay_ms(100); 

    wait_for_command();


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
    (0<<ZE1) | (0<<ZE2) | (1<<CATCHER);
  DDRB |= (1<<PB7) | (1<<PB6) | (1<<PB5);
  //^ 7 is pwm, 6 is debug
  PORTB |= (1<<ZE1) | (1<<ZE2);

  //D
  //DDRD |= (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R) 
  //^ changed to A due to tc problems
  DDRD |= (0<<TOP_Z) | (0<<BOT_Z) | (0<<CATCHER_SENSE);
  PORTD |= (1<<TOP_Z) | (1<<BOT_Z) | (1<<CATCHER_SENSE);

  //E
  DDRE |= (0<<TRANS) | (1<<RECEV) | 
    (0<<START_SIG) | (1<<END_SIG) | (0<<ORTHO);
  DDRE |= (1<<3) | (1<<4) | (1<<5); //pwm
  PORTE |= (1<<TRANS) | (1<<ORTHO) | (1<<START_SIG);

  //F
  DDRF |= (0<<IR0) | (0<<IR1) | (0<<IR2)
    | (1<<S_TRIG) | (1<<SR) | (1<<SL) | (0<<SE1) | (0<<SE2);
  PORTF |= (0<<IR0) | (0<<IR1) | (0<<IR2) | (1<<SE1) | (1<<SE2);

  //enable interrupts for INTx rising edge
  EICRA |= ( (1<<ISC01) | (1<<ISC00) );
  EICRA |= ( (1<<ISC11) | (0<<ISC10) ); //falling edge
  EICRA |= ( (1<<ISC21) | (1<<ISC20) );

  //enable interrupts
  EIMSK |= (1<<INT0) | (1<<INT1) | (1<<INT2);
}

//PD0
//sensor
ISR(INT0_vect){
  zaxis_limit_switch();
}

//PD1
ISR(INT1_vect){
  zaxis_encoder();
}

//ERNIE - There is a problem with this! It can only go up one way
//may be hardware
ISR(INT2_vect){
  //PD2
  if(bit_is_set(PINF,SE2_PIN)){
    sol_dir = -1;
    sol_position--;
  } else {
    sol_dir = 1;
    sol_position++;
  } 
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
} //end tcnt1 init

ISR(TIMER1_COMPA_vect){ 
  TCNT1 = 0; //change this to CTC mode later
  //^ CTC mode was giving me interrupt and pin errors
  stepper_timer();
}

//-----------------------------------------------------
//pwm for servo
void tcnt2_init(void){
 
 //have this set to 50Hz
  
  //enable timer/counter compare interrupt2 
  TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<COM21) | 
    (1<<COM20) | (1<<CS21) | (1<<CS20);
  //I should probably set a top somewhere  
  OCR2 = 185; //slowest for sol_motor
}
//-----------------------------------------------------

//---------------------------------------------------------------------
//DC Motor interrupt
void tcnt0_init(void){
  TCCR0 |= (1<<CS01); //1/8 prescaling should I slow this down?
  ASSR |= (0<<AS0);
  TIMSK |= (1<<TOIE0);
} //end tcnt_init

//ERNIE change this to compare vector?
ISR(TIMER0_OVF_vect){
  OCR3B = process_zaxis();

  //solinoid motor
  if(sol_des_pos == sol_position){
    solinoid_halt();   
  } 
  if(sol_des_pos > sol_position ){
    solinoid_cw(); //make sure this is actually cw
  }
  if(sol_des_pos < sol_position){
    solinoid_ccw();
  }

} //end timer0 ISR
//end motor stuff
//---------------------------------------------------------------------

//--------------------------------------------
//Catcher motor pwm
void tcnt3_init(){
  //Fast PWM, TOP: ICR3, update: BOT, TOV3 set on TOP
  //clear when (OCR3A == TCNT3), set on compare match 
  TCCR3A |= (1<<WGM31) | (1<<COM3A1) | (1<<COM3B1);
  TCCR3B |= (1<<WGM33) | (1<<WGM32); 

  //1/64 scaler
  //TCCR3B |=  (1<<CS31) | (1<<CS30);  
  TCCR3B |=  (1<<CS31); // | (1<<CS30);  

  //For scaling 
  ICR3 = 5000; //Top

  //pwm for z motor
  OCR3B = 2500; //min

  //pwm for servo
  OCR3A = 250; //spool
  //OCR3A = 375; //down
  //OCR3A = ?; //halt (we don't need this)
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
} //ISR ADC
//--------------------------------------------

void get_IR_data(){
  ADMUX &= ~ ((1<<MUX0) | (1<<MUX1)); //clearing all bits to set ADC0
  ADCSRA |= (1<<ADSC); //starts the ADC conversions
  //delay?

  ADMUX |= (1<<MUX0); //after clearing this will enable ADC1
  ADCSRA |= (1<<ADSC); //starts the ADC conversions
  //delay?

  ADMUX &= ~ ((1<<MUX0) | (1<<MUX1)); //clearing all bits to set ADC0
  ADMUX |= (1<<MUX1); //after clearing this will enable ADC1
  ADCSRA |= (1<<ADSC); //starts the ADC conversions
  //delay?
}





