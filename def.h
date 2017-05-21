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

//catcher motor
#define CATCHER PB7

//zmotor
#define ZMOTOR_R PB0 //motor control
#define ZMOTOR_L PB1
#define ZE1 PB2 //encoder
#define ZE2 PB3

//interrupt sensors
#define TOP_Z PD0
#define BOT_Z PD1
#define CATCHER_SENSE PD2

//steppers
#define STEP_L PD4 //basically a clock
#define STEP_R PD5
#define DIR_L  PD6
#define DIR_R  PD7

//UART
#define TRANS PE0
#define RECEV PE1

//signals
#define START_SIG PE6 //for game rules
#define END_SIG PE7
#define ORTHO PE2




