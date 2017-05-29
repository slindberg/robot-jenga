#include "zaxis_control.h"

PID_DATA z_pid;

// Control loop gains
#define Z_PID_KP 0.2
#define Z_PID_KI 0.825
#define Z_PID_KD 0.1

#define Z_PID_MAX_OUTPUT 5000
#define Z_PID_MIN_OUTPUT 3500

int32_t z_des_pos = 0; //where to go
int32_t z_position = 0; //reference z_position
int8_t z_dir = 0; //1 down, -1 is up, 0 is undefined
uint8_t run_zmotor = 0; //enables the motor to run or nah
//break_dir keeps track of which limit switch it hit
int8_t zbreak_dir = 0; //1 down, -1 up, 0 undefined

//gets called with timer0
uint16_t process_zaxis(){
  static uint8_t count = 0;
  static uint16_t zduty = 0;

  if(count == 0){
    zduty = stepPID(&z_pid, (float) z_des_pos, (float) z_position);
  }
  count++;

  if(run_zmotor == 1){
    //zbreak_dir = 0; //reset to 0
    if(z_des_pos == z_position){
      z_halt();
    }
    if(z_des_pos > (z_position)){
      z_up();
    }
    if(z_des_pos < (z_position)){
      z_down();
    }
  }

  //debug
  if(zbreak_dir == -1){
    PORTB &= ~(1<<5);
    PORTB |= (1<<6);
  } else if(zbreak_dir == 1){
    PORTB |= (1<<5);
    PORTB &= ~(1<<6);
  } else {
    PORTB |= (1<<5);
    PORTB |= (1<<6);
  }
  //end debug

  return zduty;
}

void home_zaxis() {
  move_zaxis_absolute(INT32_MIN);
  z_position = 0;
}

void move_zaxis(int32_t delta){
  move_zaxis_absolute(z_position + delta);
}

void move_zaxis_absolute(int32_t des_pos){
  if( !((zbreak_dir == 1) && (z_position < des_pos)) &&
      !((zbreak_dir == -1) && (z_position > des_pos)) ){
    z_des_pos = des_pos;
    run_zmotor = 1;
    zbreak_dir = 0;

    float kp = Z_PID_KP, ki = Z_PID_KI, kd = Z_PID_KD;
    if (des_pos > z_position) {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
    }
    initializePID(&z_pid, kp, ki, kd, Z_PID_MIN_OUTPUT, Z_PID_MAX_OUTPUT);
  }
}

void z_down(){
  PORTB |= (1<<ZMOTOR_R); //PB0
  PORTB &= ~(1<<ZMOTOR_L); //PB1
}

void z_up(){
  PORTB &= ~(1<<ZMOTOR_R); //PB0
  PORTB |= (1<<ZMOTOR_L);  //PB1
}

void z_halt(){
  PORTB &= ~((1<<ZMOTOR_R) | (1<<ZMOTOR_L));
  run_zmotor = 0;
}

void zaxis_limit_switch(){

  if(z_des_pos > z_position){
    zbreak_dir = 1;
  } else {
    zbreak_dir = -1;
  }
  z_des_pos = z_position;

}

void zaxis_encoder(){
  if(bit_is_set(PINB,ZE2_PIN)){
    z_dir = -1; //down
    z_position--;
  } else {
    z_dir = 1; //up
    z_position++;
  }
}
