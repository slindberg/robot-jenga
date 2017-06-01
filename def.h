//defines.h
//2017 - 05 - 16
//Made to keep all the pins organized!

//ADC
#define IR0 PF0
#define IR1 PF1
#define IR2 PF2

//solinoid
#define S_TRIG PF3 	//solinoid trigger
#define SR PF4 		//solinoid right
#define SL PF5 		//solinoid left
#define SE1 PF6		//for encoders
#define SE2 PF7
#define SE1_PIN 	6 //pind 6
#define SE2_PIN 	7 //pind 7

//catcher motor
#define CATCHER PB7

//zmotor
#define ZMOTOR_R PB0 //motor control
#define ZMOTOR_L PB1
#define ZE1 PB2 //encoder
#define ZE2 PB3
#define ZE1_PIN 	2 //pinb 2
#define ZE2_PIN 	3 //pinb 3

//interrupt sensors
#define TOP_Z PD0
#define BOT_Z PD1
#define CATCHER_SENSE PD2

//steppers
#define STEP_L PA4 //basically a clock
#define DIR_L  PA5
#define STEP_R PA6
#define DIR_R  PA7

//UART
#define TRANS PE0
#define RECEV PE1

//signals
#define START_SIG PA0 //for game rules
#define START_SIG_PIN 0




