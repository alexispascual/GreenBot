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

    if (!this->is_moving) {
        Serial.println("Driving forward");
        this->DriveForward();

    }

    if (range_sensors.GetAttitude() > this->attitude_ceil) {
      
        this->CorrectAttitude(true);
        while (range_sensors.GetAttitude() > this->neutral_attitude) {;}
        this->Stop();

    } else if (range_sensors.GetAttitude() < this->attitude_floor) {
      
        this->CorrectAttitude(false);
        while (range_sensors.GetAttitude() < this->neutral_attitude) {;}
        this->Stop();

    } else if (range_sensors.GetRoverDistance() > this->rover_distance_ceil ||
        range_sensors.GetRoverDistance() < this->rover_distance_floor) {

        this->ExecuteDistanceCorrection();

    }

}

void Greenbot::CorrectAttitude(bool direction) {

    Serial.println("Correcting Attitude");

    if (direction) { // Turn clockwise
        this->hero_message[1] = this->turning_speed;
        this->hero_message[2] = this->turning_speed + 0x80;

    } else { // Turn counter-clockwise
        this->hero_message[1] = this->turning_speed + 0x80;
        this->hero_message[2] = this->turning_speed;

    }

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_turning = true;
}

void Greenbot::ExecuteDistanceCorrection() {

    if (range_sensors.GetRoverDistance() > this->rover_distance_ceil) {
	Serial.println("Too far away from the platform");
	Serial.println("Turning towards platform");

        if (range_sensors.GetAttitude() > (this->turning_angle * -1)) {

            this->CorrectAttitude(true);
        }

        while (range_sensors.GetAttitude() > (this->turning_angle * -1)) {;}

        this->Stop();
        delay(1000);

        Serial.println("Moving forward");
        this->DriveForward();

        while (range_sensors.GetRoverDistance() > this->rover_distance_ceil) {;}

        this->Stop();
        delay(1000);

        Serial.println("Straightening up...");
        this->CorrectAttitude(false);

        while (range_sensors.GetAttitude() < this->neutral_attitude){;}
        this->Stop();

    } else if (range_sensors.GetRoverDistance() < this->rover_distance_floor) {

        Serial.println("Too close from the platform");
        Serial.println("Turning away from platform");

        if (range_sensors.GetAttitude() < this->turning_angle) {
            this->CorrectAttitude(false);
        }

        while (range_sensors.GetAttitude() < this->turning_angle) {;}

        this->Stop();
        delay(1000);

        Serial.println("Moving forward");
        this->DriveForward();

        while (range_sensors.GetRoverDistance() < this->rover_distance_floor) {;}

        this->Stop();
        delay(1000);

        Serial.println("Straightening up...");
        this->CorrectAttitude(true);

        while (range_sensors.GetAttitude() > this->neutral_attitude){;}
    }

    this->Stop();
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

    Serial.println("Stopping!");

    this->hero_message[1] = 0x00;
    this->hero_message[2] = 0x00;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->mast.writeMicroseconds(this->mast_neutral_pulse_width);

    this->is_moving = false;
    this->is_turning = false;
    this->mast_retracting = false;
    this->mast_extending = false;
}
