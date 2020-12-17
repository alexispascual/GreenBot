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

        #define TRIGGER_PIN_FRONT 7
        #define TRIGGER_PIN_REAR 5

        #define ECHO_PIN_FRONT 6
        #define ECHO_PIN_REAR 4

        uint8_t trigger_pins[NUM_SENSORS] = {TRIGGER_PIN_FRONT, TRIGGER_PIN_REAR};
        uint8_t echo_pins[NUM_SENSORS] = {ECHO_PIN_FRONT, ECHO_PIN_REAR};

        float front_duration;
        float rear_duration;
        float front_distance;
        float rear_distance;

        void Activate_Front_Sensors();
        void Activate_Rear_Sensors();
        
    public:
    
        Range_Sensors();
        void Initialize(uint8_t* trigger_pins, uint8_t* echo_pins, uint8_t num_sensors);
        float Get_Front_Distance();
        float Get_Rear_Distance();
        float* Get_Distances();
};

#endif
