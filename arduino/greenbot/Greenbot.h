#ifndef Greenbot_h
#define Greenbot_h

//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Range_Sensors.h"
#include <Servo.h>
#include <math.h>
class Greenbot {

    //--------------------------------------------------------------------------//
    //                                  Definitions                             //
    //--------------------------------------------------------------------------//
    private:

        #define START_FLAG 0x80
        #define MESSAGE_LENGTH 3
        #define MAST_PWM_PIN 8

        Servo mast;

        int8_t hero_message[3];
        int16_t mast_extension_pulse_width = 1600;
        int16_t mast_retraction_pulse_width = 1400;
        float speed;
        
        bool is_moving;
        bool is_turning;
        bool mast_extending;
        bool mast_retracting;

        Range_Sensors range_sensors;

        float front_distance;
        float rear_distance;
        float delta_theta;
        float delta_d;
        float delta_pulse;
        float sensor_gap = 54.0;
        float platform_distance = 25.0;

        float k_d = 0.1;
        float k_theta = 0.1;
  
    public:

        bool Initialize(float in_speed);

        void SetSpeed(float in_speed);
        void DriveForward();
        void DriveForwardWithSteering();
        void Greenbot::TurnIntoRow();
        void DriveBackward();
        void TurnCounterClockwise();
        void TurnClockwise();
        void ExtendMast();
        void RetractMast();
        void Stop();
};

#endif
