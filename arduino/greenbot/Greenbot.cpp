//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Greenbot.h"
//--------------------------------------------------------------------------//
//                    Pseudo-constructor                                    //
//--------------------------------------------------------------------------//

bool Greenbot::Initialize(unsigned char in_speed){

    
    this->speed = in_speed;

    this->is_moving = false;
    this->is_turning = false;
    this->mast_extending = false;
    this->mast_retracting = false;

    this->hero_message[0] = START_FLAG;

    this->mast.detach();
    this->mast.attach(MAST_PWM_PIN);
 
    Stop();

    // Somehow, Arduino does not allow try catch exception handling.
    // This will have to do.
    return true;
}

//--------------------------------------------------------------------------//
//                                  Setters                                 //
//--------------------------------------------------------------------------//
void Greenbot::SetSpeed(unsigned char in_speed) {

    this->speed = in_speed;
}
//--------------------------------------------------------------------------//
//                                  Functions                               //
//--------------------------------------------------------------------------//
void Greenbot::DriveForward(){

    this->hero_message[1] = this->speed;
    this->hero_message[2] = this->speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = true;
}

void Greenbot::DriveForwardWithSteering() {

    this->front_distance = range_sensors.Get_Front_Distance();
    this->rear_distance = range_sensors.Get_Rear_Distance();
   
    Serial.println(this->front_distance);
    Serial.println(this->rear_distance);
    
    this->delta_theta = atan((this->front_distance - this->rear_distance)/this->sensor_gap);
    this->delta_d = ((this->front_distance + this->rear_distance)/2) - this->platform_distance;
    
    Serial.println(this->delta_theta);
    Serial.println(this->delta_d);
    
    this->delta_speed = this->k_theta*this->delta_theta + this->k_d*this->delta_d;
    
    Serial.println(this->delta_speed);
    Serial.println("------------");

    this->hero_message[1] = this->speed + this->delta_speed;
    this->hero_message[2] = this->speed - this->delta_speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);
}

void Greenbot::TurnIntoRow() {

    Serial.println("Executing turning command");
    this->hero_message[1] = this->speed + this->turning_offset_speed;
    this->hero_message[2] = this->speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = true;
  
}

void Greenbot::DriveBackward(){

    this->hero_message[1] = this->speed + 0x80;
    this->hero_message[2] = this->speed + 0x80;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = true;
}

void Greenbot::TurnCounterClockwise(){

    this->hero_message[1] = this->speed + 0x80;
    this->hero_message[2] = this->speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_turning = true;
}

void Greenbot::TurnClockwise(){

    this->hero_message[1] = this->speed;
    this->hero_message[2] = this->speed + 0x80;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

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

    this->hero_message[1] = 0x00;
    this->hero_message[2] = 0x00;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = false;
    this->is_turning = false;
    this->mast_retracting = false;
    this->mast_extending = false;
}
