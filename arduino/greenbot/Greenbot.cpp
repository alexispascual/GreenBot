
//--------------------------------------------------------------------------//
//								    Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Greenbot.h"
#include <Servo.h>

//--------------------------------------------------------------------------//
//                    Pseudo-constructor                                    //
//--------------------------------------------------------------------------//

bool Greenbot::Initialize(uint8_t pwm_pins_right, uint8_t pwm_pins_left, 
                uint8_t pwm_pins_mast, float in_speed){

    this->right_wheels.detach();
    this->left_wheels.detach();
    this->mast.detach();

    this->right_wheels.attach(10);
    this->left_wheels.attach(11);
    this->mast.attach(9);

    this->in_speed = in_speed;

    this->is_moving = false;
    this->is_turning = false;
    this->mast_extending = false;
    this->mast_retracting = false;

    this->forward_pulse_width = SetForwardPulseWidth(in_speed);
    this->reverse_pulse_width = SetReversePulseWidth(in_speed);

    Stop();

    // Somehow, Arduino does not allow try catch exception handling.
    // This will have to do.
    return true;
}

//--------------------------------------------------------------------------//
//								    Setters 				   				//
//--------------------------------------------------------------------------//
void Greenbot::SetSpeed(float in_speed) {

	this->in_speed = in_speed;
	this->forward_pulse_width = SetForwardPulseWidth(in_speed);
	this->reverse_pulse_width = SetReversePulseWidth(in_speed);
}


int Greenbot::SetForwardPulseWidth(float in_speed){

	float duty_cycle = 0;

	duty_cycle = 1500 + (500 * in_speed/255);

	return (int)duty_cycle;
}

int Greenbot::SetReversePulseWidth(float in_speed){

	float duty_cycle = 0;

	duty_cycle = 1500 - (500 * in_speed/255);

    return (int)duty_cycle;
}
//--------------------------------------------------------------------------//
//								    Functions				   				//
//--------------------------------------------------------------------------//
void Greenbot::DriveForward(){

    this->right_wheels.writeMicroseconds(this->forward_pulse_width);
    this->left_wheels.writeMicroseconds(this->forward_pulse_width);

    this->is_moving = true;
}

void Greenbot::DriveBackward(){

    this->right_wheels.writeMicroseconds(this->reverse_pulse_width);
    this->left_wheels.writeMicroseconds(this->reverse_pulse_width);

    this->is_moving = true;
}

void Greenbot::TurnCounterClockwise(){

    this->right_wheels.writeMicroseconds(this->forward_pulse_width);
    this->left_wheels.writeMicroseconds(this->reverse_pulse_width);

    this->is_turning = true;
}

void Greenbot::TurnClockwise(){

    this->right_wheels.writeMicroseconds(this->reverse_pulse_width);
    this->left_wheels.writeMicroseconds(this->forward_pulse_width);

    this->is_turning = true;
}

void Greenbot::ExtendMast(){

    this->mast_retracting = false;

    if (this->mast_extending == false) {

        this->mast.writeMicroseconds(this->mast_extension_pulse_width);

        this->mast_extending = true;
    }
}

void Greenbot::RetractMast(){

    this->mast_extending = false;

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
