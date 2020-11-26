#include "Arduino.h"
#include "Range_Sensors.h"

//--------------------------------------------------------------------------//
//                   Constructor                                            //
//--------------------------------------------------------------------------//
/*
Range_Sensors::Range_Sensors(uint8_t* trigger_pins, uint8_t* echo_pins, uint8_t num_sensors) {
    
    for (byte i = 0; i < num_sensors; i++) {

        this->trigger_pins[i] = trigger_pins[i];
        this->echo_pins[i] = echo_pins[i];
        
        pinMode(trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(echo_pins[i], INPUT); // Sets the echoPin as an INPUT

    }
}*/

void Range_Sensors::Initialize() {

  for (byte i = 0; i < NUM_SENSORS; i++) {
      
        pinMode(this->trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(this->echo_pins[i], INPUT); // Sets the echoPin as an INPUT

    }
}



void Range_Sensors::Activate_Sensors() {
    Serial.println("Activating Sensors...");

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(this->trigger_pins[i], LOW); // Clears the trigPin condition
    }

    delayMicroseconds(2);

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(this->trigger_pins[i], HIGH); // Set trigger pin high for 10 micro seconds
    }

    delayMicroseconds(10);

    for (byte i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(this->trigger_pins[i], LOW); // Clear the trigger pin
    }

    this->front_duration = pulseIn(this->echo_pins[ECHO_PIN_FRONT], HIGH);// Reads the echoPin, returns the sound wave travel time in microseconds
    this->rear_duration = pulseIn(this->echo_pins[ECHO_PIN_REAR], HIGH);
    
    Serial.println(this->front_duration);
    Serial.println(this->rear_duration);
    
    this->front_distance = this->front_duration * 0.034/2;
    this->rear_distance = this->rear_duration * 0.034/2;
    
}

float Range_Sensors::Get_Front_Distance() {
    Activate_Sensors();
    return this->front_distance;

}

float Range_Sensors::Get_Rear_Distance() {
    Activate_Sensors();
    return this->rear_distance;

}

float* Range_Sensors::Get_Distances() {
    Activate_Sensors();

    static float distances[NUM_SENSORS];
    distances[0] = this->front_distance;
    distances[1] = this->rear_distance;

    return distances;
}
