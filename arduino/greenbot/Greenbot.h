#ifndef Greenbot_h
#define Greenbot_h

//--------------------------------------------------------------------------//
//								    Imports					   				//
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include <Servo.h>
class Greenbot {

	//--------------------------------------------------------------------------//
	//								    Definitions					   				//
	//--------------------------------------------------------------------------//
	private:

	#define PWM_PIN_RIGHT 10
	#define PWM_PIN_LEFT 11
  #define PWM_PIM_MAST 9

  Servo right_wheels;
  Servo left_wheels;
  Servo mast;
  
	uint8_t speed;
	uint16_t forward_pulse_width;
	uint16_t reverse_pulse_width;
  uint16_t neutral_pulse_width = 1500;

	bool is_moving;
	bool is_turning;
  
	public:

	Greenbot(uint8_t pwm_pins_right, 
			uint8_t pwm_pins_left,
      uint8_t pwm_pins_mast, 
			uint8_t speed);

	void SetSpeed(uint8_t speed);
	uint16_t SetForwardPulseWidth(uint8_t speed);
	uint16_t SetReversePulseWidth(uint8_t speed);
	void DriveForward();
	void DriveBackward();
	void Stop();
};

#endif