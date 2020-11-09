
//--------------------------------------------------------------------------//
//								    Imports					   				//
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Greenbot.h"
#include <Servo.h>

//--------------------------------------------------------------------------//
//								    Constructor				   				//
//--------------------------------------------------------------------------//


	Greenbot::Greenbot(uint8_t pwm_pins_right, uint8_t pwm_pins_left, 
	                  uint8_t pwm_pins_mast, uint8_t speed){

		this->right_wheels.attach(pwm_pins_right);
    this->left_wheels.attach(pwm_pins_left);
    this->mast.attach(pwm_pins_mast);
    
		this->speed = speed;

		this->is_moving = false;
		this->is_turning = false;
    this->mast_extending = false;
    this->mast_retracting = false;

		this->forward_pulse_width = SetForwardPulseWidth(speed);
		this->reverse_pulse_width = SetReversePulseWidth(speed);


	}
//--------------------------------------------------------------------------//
//								    Setters 				   				//
//--------------------------------------------------------------------------//
	void Greenbot::SetSpeed(uint8_t speed) {

		this->speed = speed;
		this->forward_pulse_width = SetForwardPulseWidth(speed);
		this->reverse_pulse_width = SetReversePulseWidth(speed);

	}


	uint16_t Greenbot::SetForwardPulseWidth(uint8_t speed){

		float duty_cycle = 0;

		duty_cycle = 1500 + ((500/255) * speed);

  	return (uint16_t)duty_cycle;

	}

	uint16_t Greenbot::SetReversePulseWidth(uint8_t speed){

		float duty_cycle = 0;

		duty_cycle = 1000 + ((500/255)*speed);

    return (uint16_t)duty_cycle;

	}
//--------------------------------------------------------------------------//
//								    Functions				   				//
//--------------------------------------------------------------------------//
	void Greenbot::DriveForward(){

		if (this->is_moving == false) {

			this->right_wheels.writeMicroseconds(this->forward_pulse_width);
			this->left_wheels.writeMicroseconds(this->forward_pulse_width);

     this->is_moving = true;

		} else {

			Serial.println("Already moving forward");
		}
	}

	void Greenbot::DriveBackward(){

		if (this->is_moving == false) {

			this->right_wheels.writeMicroseconds(this->reverse_pulse_width);
      this->left_wheels.writeMicroseconds(this->reverse_pulse_width);

      this->is_moving = true;

		} else {

			Serial.println("Already moving backwards");
		}
	}

 void Greenbot::TurnCounterClockwise(){

    if (this->is_turning == false) {

      this->right_wheels.writeMicroseconds(this->forward_pulse_width);
      this->left_wheels.writeMicroseconds(this->reverse_pulse_width);

      this->is_turning = true;

    } else {

      Serial.println("Already turning counter clockwise");
    }
  }

  void Greenbot::TurnClockwise(){

    if (this->is_turning == false) {

      this->right_wheels.writeMicroseconds(this->reverse_pulse_width);
      this->left_wheels.writeMicroseconds(this->forward_pulse_width);

      this->is_turning = true;

    } else {

      Serial.println("Already turning clockwise");
    }
  }

  void Greenbot::ExtendMast(){

    if (this->mast_extending == false) {

      this->mast.writeMicroseconds(this->mast_extension_pulse_width);

      this->mast_extending = true;

    }
  }

  void Greenbot::RetractMast(){

    if (this->mast_retracting == false) {

      this->mast.writeMicroseconds(this->mast_retraction_pulse_width);

      this->mast_retracting = true;

    }
    
  }

	void Greenbot::Stop(){
      
      this->right_wheels.writeMicroseconds(this->neutral_pulse_width);
      this->left_wheels.writeMicroseconds(this->neutral_pulse_width);
      this->mast.writeMicroseconds(this->neutral_pulse_width);

      this->is_moving = false;
      this->is_turning = false;
      this->mast_retracting = false;
      this->mast_extending = false;
      
	}
