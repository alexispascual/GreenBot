#ifndef Greenbot_h
#define Greenbot_h

//--------------------------------------------------------------------------//
//								    Imports					   				//
//--------------------------------------------------------------------------//

#include "Arduino.h"

class Greenbot {

	//--------------------------------------------------------------------------//
	//								    Definitions					   				//
	//--------------------------------------------------------------------------//
	private:

	#define PWM_PIN_3 3
	#define PWM_PIN_5 5

	uint8_t pwm_pins_right;
	uint8_t pwm_pins_left;
	uint8_t speed;
	uint8_t forward_duty_cycle;
	uint8_t reverse_duty_cycle;

	bool is_moving;
	bool is_turning;

	public:

	Greenbot(uint8_t pwm_pins_right, 
			uint8_t pwm_pins_left, 
			uint8_t speed);
}
