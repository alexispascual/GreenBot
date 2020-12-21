#include "Arduino.h"
#include "Range_Sensors.h"

//--------------------------------------------------------------------------//
//                   Pseudo-Constructor                                     //
//              Arduino is being weird with memory intializations           //
//--------------------------------------------------------------------------//

Range_Sensors::Range_Sensors (){

    for (byte i = 0; i < NUM_SENSORS; i++) {
        
        pinMode(this->trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(this->echo_pins[i], INPUT); // Sets the echoPin as an INPUT

    }
}

void Range_Sensors::Initialize(uint8_t* trigger_pins, uint8_t* echo_pins, uint8_t num_sensors) {

    for (byte i = 0; i < num_sensors; i++) {

        this->trigger_pins[i] = trigger_pins[i];
        this->echo_pins[i] = echo_pins[i];
        
        pinMode(this->trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(this->echo_pins[i], INPUT); // Sets the echoPin as an INPUT

    }
}

void Range_Sensors::ActivateFrontSensors() {

    digitalWrite(this->trigger_pins[0], LOW);
    delayMicroseconds(2);
    
    digitalWrite(this->trigger_pins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigger_pins[0], LOW);

    this->front_duration = pulseIn(this->echo_pins[0], HIGH);
    this->front_distance = floor(this->front_duration * 0.034/2);
   
}

void Range_Sensors::ActivateRearSensors() {

    digitalWrite(this->trigger_pins[1], LOW);
    delayMicroseconds(2);

    digitalWrite(this->trigger_pins[1], HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigger_pins[1], LOW);

    this->rear_duration = pulseIn(this->echo_pins[1], HIGH);
    this->rear_distance = floor(this->rear_duration * 0.034/2);

}

float Range_Sensors::GetFrontDistance() {
    this->ActivateFrontSensors();
    return this->front_distance;
}

float Range_Sensors::GetRearDistance() {
    this->ActivateRearSensors();
    return this->rear_distance;
}

float* Range_Sensors::GetDistances() {
    this->ActivateFrontSensors();
    this->ActivateRearSensors();

    static float distances[NUM_SENSORS];
    distances[0] = this->front_distance;
    distances[1] = this->rear_distance;

    return distances;
}

float Range_Sensors::GetAttitude() {

    this->ActivateFrontSensors();
    this->ActivateRearSensors();

    return atan((this->front_distance - this->rear_distance)/this->sensor_gap);
}

float Range_Sensors::GetRoverDistance() {

    this->ActivateFrontSensors();
    this->ActivateRearSensors();

    return ((this->front_distance + this->rear_distance)/2) - this->platform_distance;
}
