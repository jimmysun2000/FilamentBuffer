#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <TMCStepper.h>
#include <Arduino.h>
#include <EEPROM.h>

#define HALL1       		PB2 		// Hall Sensor 3
#define HALL2       		PB3 		// Hall Sensor 2   
#define HALL3       		PB4 		// Hall Sensor 1

#define ENDSTOP_3   		PB7 		// Filament Detection Sensor

#define KEY_REVERSE 		PB11 		// Reverse
#define KEY_FORWARD 		PB10 		// Forward

#define LED_REVERSE 		PA5 		// Reverse
#define LED_FORWARD 		PA4 		// Forward

#define EN_PIN      		PA6 		// Stepper Enable
#define DIR_PIN     		PA7 		// Stepper Direction
#define STEP_PIN    		PC13 		// Stepper Step
#define UART        		PB1 		// UART Port

#define FILAMENT_OUTPUT  	PB15 		// Filament Detection Output
#define ERR_LED     		PA2 		// Error LED
#define STATUS_LED   		PA3  		// Status LED

#define DRIVER_ADDRESS 		0b00 		// TMC Driver address according to MS1 and MS2
#define R_SENSE 			0.11f 		// Match to your driver

#define SPEED       		300  		// Rotation speed (r/min)
#define Move_Divide_NUM	((int32_t)(64))	// Micro Stepping
#define VACTRUAL_VALUE (uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715)   // VACTUAL Register Value

#define STOP 				0			// Stop
#define I_CURRENT 			(600)		// Current
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)// Enable Pin Write
#define FORWARD				1			// Filament Direction
#define BACK				0

#define DEBUG 				0

// Input states
typedef struct Buffer {
	bool buffer1_pos1_sensor_state;	
	bool buffer1_pos2_sensor_state;		
	bool buffer1_pos3_sensor_state;		
	bool buffer1_material_swtich_state;	
	bool key_reverse;
	bool key_forward;
} Buffer;

// Output stepper states
typedef enum {
	Forward = 0,	// Forward
	Stop,			// Stop
	Back			// Reverse
} Motor_State;

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state(void);
extern void motor_control(void);

extern void buffer_init();
extern void buffer_loop(void);
extern void timer_it_callback();
extern void buffer_debug(void);

extern bool is_error;
extern uint32_t front_time;	// Forward time
extern uint32_t timeout;
extern bool is_front;
extern TMC2209Stepper driver;

#endif