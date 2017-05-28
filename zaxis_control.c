#include "zaxis_control.h"

PID z_pid;
int32_t z_des_pos = 0; //where to go
int32_t z_position = 0; //reference z_position
int8_t z_dir = 0;

//gets called with timer0
uint16_t process_zaxis(){
  static uint8_t count = 0;
  static int32_t last_z_pos = 0;
  static uint16_t zduty = 0;

  //zmotor
  if(z_des_pos == z_position){
    z_halt();   
  } 
  if(z_des_pos > (z_position)){
    z_up();
  }
  if(z_des_pos < (z_position)){
    z_down();
  }

  //ERNIE Get PID contorl to work!!!
  //make sure all the equations are good!!!
  if(count == 0){ 
    int32_t delta_pos = (z_position - last_z_pos); 
    float zspeed = (delta_pos* 0.06144);
    //should I abs this or nah? also should this just be des_pos?

    int32_t set_point = (z_des_pos - z_position);	
    zduty = pid_update(&z_pid, set_point, zspeed);
    last_z_pos = z_position;
  }
  count++;

  return zduty;
}

void move_zaxis(int32_t des_pos){
  z_des_pos = des_pos;
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

void zaxis_limit_switch(){
  //ERNIE make this actually stop and go up/down
  //a little when triggered

  //if(z_dir somethign something 
  z_des_pos = z_position;
}

void zaxis_encoder(){
  if(bit_is_set(PINB,ZE2_PIN)){
    z_dir = -1;
    z_position--;	    
  } else {
    z_dir = 1;
    z_position++;	
  } 
}

void zpid_init(){
  z_pid.min = 2500;
  z_pid.max = 5000;
  z_pid.proportional_const = 0.5; 
  z_pid.intergral_const = 0.00; 
  z_pid.derivative_const = 0.00;
}

//I hopw floats work
uint16_t pid_update(PID* pid, float set_point, float proccess_var){

  return 4000;


  float prev_error; //error
  float manp; //manupulate (sp) value?
  float tmpi; //temp intergral value?

  prev_error = pid->error;
  pid->error = (set_point - proccess_var);
  tmpi = pid->intergral_value + pid->error;
  //bound the intergral
  manp = (pid->proportional_const + 
      pid->intergral_const) + 
    pid->error + tmpi + 
    pid->derivative_const*(prev_error-pid->error);

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



