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
  
	float in_speed;
	int forward_pulse_width;
	int reverse_pulse_width;
  int neutral_pulse_width = 1500;
  int mast_extension_pulse_width = 1600;
  int mast_retraction_pulse_width = 1400;

	bool is_moving;
	bool is_turning;
  bool mast_extending;
  bool mast_retracting;
  
	public:

	bool Initialize(uint8_t pwm_pins_right, 
			uint8_t pwm_pins_left,
      uint8_t pwm_pins_mast, 
			float in_speed);

	void SetSpeed(float in_speed);
	int SetForwardPulseWidth(float in_speed);
	int SetReversePulseWidth(float in_speed);
	void DriveForward();
	void DriveBackward();
  void TurnCounterClockwise();
  void TurnClockwise();
  void ExtendMast();
  void RetractMast();
	void Stop();
};

#endif
