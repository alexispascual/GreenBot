#ifndef Range_Sensors_h
#define Range_Sensors_h

//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"

class Range_Sensors {
//--------------------------------------------------------------------------//
//                                  Definitions                             //
//--------------------------------------------------------------------------//
    private:

        #define NUM_SENSORS 2

        #define TRIGGER_PIN_FRONT 9
        #define TRIGGER_PIN_REAR 5

        #define ECHO_PIN_FRONT 8
        #define ECHO_PIN_REAR 4

        uint8_t trigger_pins[NUM_SENSORS] = {TRIGGER_PIN_FRONT, TRIGGER_PIN_REAR};
        uint8_t echo_pins[NUM_SENSORS] = {ECHO_PIN_FRONT, ECHO_PIN_REAR};

        float front_duration;
        float rear_duration;
        float front_distance;
        float rear_distance;
        float sensor_gap = 100.0;
        float platform_distance = 45.0;

        void ActivateFrontSensors();
        void ActivateRearSensors();
        
    public:
    
        Range_Sensors();
        void Initialize(uint8_t* trigger_pins, uint8_t* echo_pins, uint8_t num_sensors);
        float GetFrontDistance();
        float GetRearDistance();
        float* GetDistances();
        float GetAttitude();
        float GetRoverDistance();
};

#endif
