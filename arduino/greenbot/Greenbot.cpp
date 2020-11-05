
//--------------------------------------------------------------------------//
//								    Imports					   				//
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Greenbot.h"

//--------------------------------------------------------------------------//
//								    Constructor				   				//
//--------------------------------------------------------------------------//


	Greenbot::Greenbot(uint8_t pwm_pins_right, uint8_t pwm_pins_left, uint8_t speed){

		this->pwm_pins_right = pwm_pins_right;
		this->pwm_pins_left = pwm_pins_left;
		this->speed = speed;

		this->is_moving = false;
		this->is_turning = false;

		this->forward_duty_cycle = SetForwardDutyCycle(speed);
		this->reverse_duty_cycle = SetReverseDutyCycle(speed);

		pinMode(pwm_pins_right, OUTPUT);
		pinMode(pwm_pins_left, OUTPUT);

	}
//--------------------------------------------------------------------------//
//								    Setters 				   				//
//--------------------------------------------------------------------------//
	void Greenbot::SetSpeed(uint8_t speed) {

		this->speed = speed;
		this->forward_duty_cycle = SetForwardDutyCycle(speed);
		this->reverse_duty_cycle = SetReverseDutyCycle(speed);

	}


	uint8_t Greenbot::SetForwardDutyCycle(uint8_t speed){

		float duty_cycle = 0;

		duty_cycle = (speed + 255) / 2;

  		return (uint8_t)duty_cycle;

	}

	uint8_t Greenbot::SetReverseDutyCycle(uint8_t speed){

		float duty_cycle = 0;

		duty_cycle = (-speed + 255) / 2;

  		return (uint8_t)duty_cycle;

	}
//--------------------------------------------------------------------------//
//								    Functions				   				//
//--------------------------------------------------------------------------//
	void Greenbot::DriveForward(){

		this->is_moving = true;

		if (this->is_moving == false) {

			analogWrite(this->pwm_pins_left, this->forward_duty_cycle);
			analogWrite(this->pwm_pins_right, this->forward_duty_cycle);

		} else {

			Serial.println("Already moving forward");
		}
	}

	void Greenbot::DriveBackward(){

		this->is_moving = true;

		if (this->is_moving == false) {

			analogWrite(this->pwm_pins_left, this->reverse_duty_cycle);
			analogWrite(this->pwm_pins_right, this->reverse_duty_cycle);

		} else {

			Serial.println("Already moving forward");
		}
	}

	void Greenbot::Stop(){

		this->is_moving = false;

		analogWrite(this->pwm_pins_left, 0);
		analogWrite(this->pwm_pins_right, 0);
		
	}


