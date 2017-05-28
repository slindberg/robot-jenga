//2017 - 05 - 16

#include "motor_functions.h"
#define STEP_PORT PORTA
#define STEP_INTERVAL 100


void move_stepper(StepperState *state) {
  if (!state->movement || state->interval_index >= state->movement->length) {
    return;
  }

  // Clear the travel pin to reset pulse state
  STEP_PORT &= ~state->travel_mask;

  // If we're at the start of an interval, initialize remaining steps
  if (state->interval_count == 0) {
    int8_t delta = state->movement->intervals[state->interval_index];

    // Set direction pin, high is clockwise
    if (delta < 0) {
      STEP_PORT |= state->direction_mask;
      state->steps_remaining = -delta;
    } else {
      STEP_PORT &= ~state->direction_mask;
      state->steps_remaining = delta;
    }
  }

  if (state->steps_remaining != 0) {
    // If the call delay is zero, we've just completed a step, so calculate a new
    // delay. This is done to ensure that all steps are completed within the interval,
    // since integer division would result in missed steps.
    if (state->call_delay == 0) {
      state->call_delay = (STEP_INTERVAL - state->interval_count) / state->steps_remaining;
    }

    // If the enough timer calls have been made, pulse the stepper and reset state vars
    if (state->delay_count == state->call_delay) {
      STEP_PORT |= state->travel_mask;
      state->steps_remaining--;
      state->call_delay = 0;
      state->delay_count = 0;
    } else {
      state->delay_count++;
    }
  }

  // At the end of an interval, reset counts and advance to the next interval,
  if (state->interval_count == STEP_INTERVAL) {
    state->interval_index++;
    state->interval_count = 0;
    state->delay_count = 0;
  } else {
    state->interval_count++;
  }
}

void reset_stepper_state(StepperState *state, StepperMovement *movement) {
    state->movement = movement;
    state->interval_index = 0;
    state->steps_remaining = 0;
    state->delay_count = 0;
    state->interval_count = 0;
}


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



