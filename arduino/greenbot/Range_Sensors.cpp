#include "Arduino.h"
#include "Range_Sensors.h"

//--------------------------------------------------------------------------//
//                   Constructor                                            //
//--------------------------------------------------------------------------//

Range_Sensors::Range_Sensors(uint8_t* trigger_pins, uint8_t* echo_pins, uint8_t num_sensors) {
    
    for (byte i = 0; i < NUM_SENSORS; i++) {

        pinMode(trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(echo_pins[i], INPUT); // Sets the echoPin as an INPUT

    }

}

void Range_Sensors::Activate_Sensors() {

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(this->trigger_pins[i], LOW); // Clears the trigPin condition
    }

    delayMicroseconds(2);

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(trigger_pins[i], HIGH); // Set trigger pin high for 10 micro seconds
    }

    delayMicroseconds(10);

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(trigger_pins[i], LOW); // Clear the trigger pin
    }

    this->front_duration = pulseIn(echo_pins[ECHO_PIN_FRONT], HIGH);// Reads the echoPin, returns the sound wave travel time in microseconds
    this->rear_duration = pulseIn(echo_pins[ECHO_PIN_REAR], HIGH);

    this->front_distance = this->front_duration * 0.034/2;
    this->rear_distance = this->rear_duration * 0.034/2;
    
}

float Range_Sensors::Get_Front_Distance() {

    return this->front_distance;

}

float Range_Sensors::Get_Rear_Distance() {

    return this->rear_distance;

}

float* Range_Sensors::Get_Distances() {

    static float distances[NUM_SENSORS];
    distances[0] = this->front_distance;
    distances[1] = this->rear_distance;

    return distances;
}