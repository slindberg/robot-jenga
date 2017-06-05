/* Ernie Bodle & Steven Lindberg
 * Started on 2017 - 05 - 11
 * robot.c
 * This lab uses a modified version of my lab4 ECE473 code.
 *
 * For ROB 421 project
 * Jenga Robot
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart_functions.h"
#include "commands.h"
#include "arm_control.h"
#include "zaxis_control.h"
#include "ef_rotation_control.h"
#include "catcher_control.h"
#include "def.h"

//--------------------------
//ADC
uint16_t adc_num = 0; //get rid of this
uint16_t IR0_num = 0; 	//this is the number outputted by the adc
uint16_t IR1_num = 0; 	//this is the number outputted by the adc
uint16_t IR2_num = 0; 	//this is the number outputted by the adc
void adc_init();
ISR(ADC_vect);
//--------------------------

void tcnt0_init(void);
ISR(TIMER0_OVF_vect);
//--------------------------

//sensor interrupts
ISR(INT0_vect);
ISR(INT1_vect);
ISR(INT2_vect);
ISR(INT3_vect);

void rob_init();

//--------------------------

// arm stepper motors
void tcnt2_init(); //steppers
ISR(TIMER1_COMPA_vect);

//for pwm
void tcnt1_init(); //dc motor pwm
void tcnt3_init(); //servo pwm

void get_IR_data();

int main() {
  //setting up pins
  rob_init();

  //interrupts
  tcnt0_init(); //DC motor control
  tcnt1_init(); //DC motor pwm
  tcnt2_init(); //stepper motors
  tcnt3_init(); //servo motor pwm

  //adc_init(); //we only need this if we are using IR sensors
  uart_init(); //uart
  sei(); //enable all interrupts before entering loop

  // PID control initilization
  init_zaxis_pid();
  init_ef_pid();

  while(1) {
    wait_for_command();
  }

  return 0; //do I need this in a uC?
}

void rob_init(){
  //A
  DDRA |=  (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R);
  PORTA |= (1<<START_SIG);

  //B
  DDRB |= (1<<ZMOTOR_R) | (1<<ZMOTOR_L) |
    (0<<ZE1) | (0<<ZE2) | (1<<CATCHER);
  //5, 6, 7 are PWM for DC motors
  DDRB |= (1<<PB5) | (1<<PB6) | (1<<PB7);
  PORTB |= (1<<ZE1) | (1<<ZE2);

  //D
  //DDRD |= (1<<STEP_L) | (1<<STEP_R) | (1<<DIR_L) | (1<<DIR_R)
  //^ changed to A due to tc problems
  DDRD |= (0<<TOP_Z) | (0<<BOT_Z) | (0<<CATCHER_SENSE);
  PORTD |= (1<<TOP_Z) | (1<<BOT_Z) | (1<<CATCHER_SENSE);

  //E
  DDRE |= (0<<TRANS) | (1<<RECEV);
  DDRE |= (1<<3) | (1<<4) | (1<<5); //pwm
  PORTE |= (1<<TRANS);

  //F
  DDRF |= (0<<IR0) | (0<<IR1) | (0<<IR2)
    | (1<<S_TRIG) | (1<<SR) | (1<<SL) | (0<<SE1) | (0<<SE2);
  PORTF |= (0<<IR0) | (0<<IR1) | (0<<IR2) | (1<<SE1) | (1<<SE2);

  //enable interrupts for INTx rising edge
  EICRA |= ( (1<<ISC01) | (1<<ISC00) );
  EICRA |= ( (1<<ISC11) | (0<<ISC10) ); //falling edge
  EICRA |= ( (1<<ISC21) | (0<<ISC20) );
  EICRA |= ( (1<<ISC31) | (1<<ISC30) );

  //enable interrupts
  EIMSK |= (1<<INT0) | (1<<INT1) | (1<<INT2) | (1<<INT3);
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
  ef_rotation_encoder();
}

ISR(INT3_vect){
  catcher_limit_switch();
}

//--------------------------------------------------
// DC motor pwm
void tcnt1_init(void){
  //Fast PWM, TOP: ICR3, update: BOT, TOV3 set on TOP
  //clear when (OCR3A == TCNT3), set on compare match
  TCCR1A |= (1<<WGM11) | (1<<COM1C1) | (1<<COM1B1);
  TCCR1B |= (1<<WGM13) | (1<<WGM12);
  //no prescaling
  TCCR1B |=  (1<<CS10);
  //For scaling
  ICR1 = 5000; //Top

  //pwm for ef motor
  OCR1C = 0;

  //pwm for z motor
  OCR1B = 0;
} //end tcnt1 init

//--------------------------------------------------
//stepper motor functions
void tcnt2_init(void){
  TCCR2 |= (1<<CS21) | (1<<CS20); //clk/64 mode
  TIMSK |= (1<<OCIE2);
  OCR2 = 0x40; //64
}

ISR(TIMER2_COMP_vect){
  TCNT2 = 0; //change this to CTC mode later
  //^ CTC mode was giving me interrupt and pin errors
  stepper_timer();
}

//--------------------------------------------------
//DC Motor interrupt
void tcnt0_init(void){
  TCCR0 |= (1<<CS01); //1/8 prescaling should I slow this down?
  ASSR |= (0<<AS0);
  TIMSK |= (1<<TOIE0);
} //end tcnt_init

ISR(TIMER0_OVF_vect){
  OCR1B = process_zaxis();
  OCR1C = process_ef_rotation();
} //end timer0 ISR


//--------------------------------------------------
//Catcher motor pwm
void tcnt3_init(){
  //Fast PWM, TOP: ICR3, update: BOT, TOV3 set on TOP
  //clear when (OCR3A == TCNT3), set on compare match
  TCCR3A |= (1<<WGM31) | (1<<COM3A1);
  TCCR3B |= (1<<WGM33) | (1<<WGM32);

  TCCR3B |=  (1<<CS31) | (1<<CS30); //1/64

  //For scaling
  ICR3 = 5000; //Top

  //pwm for servo
  OCR3A = 250; //spool
  //OCR3A = 375; //down
} //tcnt3_init

//--------------------------------------------------
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





