//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Greenbot.h"
//--------------------------------------------------------------------------//
//                    Pseudo-constructor                                    //
//--------------------------------------------------------------------------//

Greenbot_IMU *temp_greenbot_IMU; //defining global for the interrupt handler trick.

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

    //enable Arduino interrupt detection
    temp_greenbot_IMU = &greenbot_IMU;

    Serial.print(F("Enabling interrupt detection -- Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), this->ISR_Handler, RISING);

    return greenbot_IMU.Initialize();    
}

//--------------------------------------------------------------------------//
//                                  Setters                                 //
//--------------------------------------------------------------------------//
void Greenbot::SetSpeed(unsigned char in_speed) {

    Serial.print("Updating Greenbot Speed: ");
    Serial.println(in_speed);

    this->speed = in_speed;
}
//--------------------------------------------------------------------------//
//                                  Functions                               //
//--------------------------------------------------------------------------//
void Greenbot::DriveForward(){

    if (this->is_turning){this->Stop();}

    Serial.println("Driving forward");

    this->hero_message[1] = this->speed;
    this->hero_message[2] = this->speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = true;
}

void Greenbot::DriveForwardWithSteering() {

    if (!this->is_moving) {
        Serial.println("Driving forward with steering");
        this->DriveForward();

    }

    if (range_sensors.GetAttitude() > this->attitude_ceil) {
        
        this->Stop();
        this->CorrectAttitude(true);
        while (range_sensors.GetAttitude() > this->neutral_attitude) {;}
        this->Stop();

    } else if (range_sensors.GetAttitude() < this->attitude_floor) {
        
        this->Stop();
        this->CorrectAttitude(false);
        while (range_sensors.GetAttitude() < this->neutral_attitude) {;}
        this->Stop();

    } else if (range_sensors.GetRoverDistance() > this->rover_distance_ceil ||
        range_sensors.GetRoverDistance() < this->rover_distance_floor) {

        this->Stop();
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

    Serial.println("Executing Distance Correction");

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

void Greenbot::Turn90Degrees() { 

    float temp_yaw = 0;

    if (greenbot_IMU.GetDeviceStatus() == 0) { // Device successfully initialized

        Serial.println("Turning 90 Degrees!");

        Serial.print("Current yaw: ");
        this->current_yaw_deg = greenbot_IMU.GetYaw();

        if (this->current_yaw_deg > 273) { // Offset 90 degree turn by 3 degrees

            this->TurnClockwise();

            while(true){

                temp_yaw = greenbot_IMU.GetYaw();
                if (temp_yaw > 273 && temp_yaw < 360) temp_yaw -= 360;
                if (temp_yaw > (this->current_yaw_deg - 273)) break;
            }

            this->Stop();

        } else {

            this->TurnClockwise();
            while(greenbot_IMU.GetYaw() < this->current_yaw_deg + 87); // Offset 90 degree turn by 3 degrees
            this->Stop();
        }
    } else {

        Serial.println("Device initialization failed. Can't turn 90 degrees.");
    }
}

void Greenbot::DriveBackward(){

    if (this->is_turning){this->Stop();}

    Serial.println("Reversing");

    this->hero_message[1] = this->speed + 0x80;
    this->hero_message[2] = this->speed + 0x80;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_moving = true;
}

void Greenbot::TurnCounterClockwise(){

    if (this->is_moving){this->Stop();}

    Serial.println("Turning counter-clockwise!");

    this->hero_message[1] = this->speed + 0x80;
    this->hero_message[2] = this->speed;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_turning = true;
}

void Greenbot::TurnClockwise(){

    if (this->is_moving){this->Stop();}

    Serial.println("Turning clockwise!");

    this->hero_message[1] = this->speed;
    this->hero_message[2] = this->speed + 0x80;

    Serial1.write(this->hero_message, MESSAGE_LENGTH);

    this->is_turning = true;
}

void Greenbot::ExtendMast(){

    this->mast_retracting = false;

    Serial.println("Extending mast \n");

    if (this->mast_extending == false) {

        this->mast.writeMicroseconds(this->mast_extension_pulse_width);

        this->mast_extending = true;
    }
}

void Greenbot::RetractMast(){

    this->mast_extending = false;

    Serial.println("Retracting mast \n");

        if (this->mast_retracting == false) {

        this->mast.writeMicroseconds(this->mast_retraction_pulse_width);

        this->mast_retracting = true;
    }
}

void Greenbot::Stop(){

    if (this->is_moving || this->is_turning || this->mast_retracting || this->mast_extending) {

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
}

//--------------------------------------------------------------------------//
//                        Interrupt Handler                                 //
//--------------------------------------------------------------------------//

static void Greenbot::ISR_Handler() {

    temp_greenbot_IMU->ISR_Handler();

}
