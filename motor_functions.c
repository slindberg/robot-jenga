//2017 - 05 - 16

#include "motor_functions.h"

void z_down(){
   z_halt(); 
    PORTB |= (1<<ZMOTOR_R); //PB0
    PORTB &= ~(1<<ZMOTOR_L); //PB1
}

void z_up(){
   z_halt(); 
    PORTB &= ~(1<<ZMOTOR_R); //PB0
    PORTB |= (1<<ZMOTOR_L);  //PB1
}

void z_halt(){
    PORTB &= ~((1<<ZMOTOR_R) | (1<<ZMOTOR_L));
}

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


void pid_init(PID *pid, int16_t min, int16_t max){
    pid->min = min;
    pid->max = max;

    //div by 10,000
    pid->proportional_const = 500; 
    pid->intergral_const = 0; 
    pid->derivative_const = 0;
}

int16_t pid_update(PID* pid, int16_t set_point, int16_t proccess_var){
    int16_t prev_error; //error
    int16_t manp; //manupulate (sp) value?
    int16_t tmpi; //temp intergral value?
    
    prev_error = pid->error;
    pid->error = (set_point - proccess_var);
    tmpi = pid->intergral_value + pid->error;
    //bound the intergral
    manp = (pid->proportional_const + 
	pid->intergral_const)/10000 + 
	pid->error + tmpi + 
	pid->derivative_const*(prev_error-pid->error)/10000;

    if((manp < pid->max) && (manp > pid->min)){
	pid->intergral_value = tmpi;	
    } else if(manp > pid->max){
	manp = pid->max;
    } else if(manp < pid->min){
	manp = pid->min;
    }
    return manp;
}

//dt can be removed if we put it in a timer counter!!!
/*void pid_update(PID* pid, int16_t curr_error, int16_t dt){
    int16_t diff;
    int16_t p_term;
    int16_t i_term;
    int16_t d_term;

    pid->int_error += (curr_error *dt);
    if(pid->int_error < -1*(pid->windup_guard))
	pid->int_error = -1*(pid->windup_guard);

    //differentiation
    diff = ((curr_error - pid->prev_error) * dt);

    //scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->intergral_gain * pid->int_error);
    d_term = (pid->derivative_gain * diff);

    //sumation of terms
    pid->control = p_term + i_term + d_term;

    //save current error as previous error for next iteration
    pid->prev_error = curr_error;
}*/ //end pid_update



