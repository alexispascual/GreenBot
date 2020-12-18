#ifndef Greenbot_h
#define Greenbot_h

//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"
#include "Range_Sensors.h"
#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>

class Greenbot {

    //--------------------------------------------------------------------------//
    //                                  Definitions                             //
    //--------------------------------------------------------------------------//
    private:

        #define START_FLAG 0x80
        #define MESSAGE_LENGTH 3
        #define MAST_PWM_PIN 10

        Servo mast;

        unsigned char hero_message[3];
        unsigned char speed;
        int16_t mast_extension_pulse_width = 1600;
        int16_t mast_retraction_pulse_width = 1400;
        
        bool is_moving;
        bool is_turning;
        bool mast_extending;
        bool mast_retracting;

        Range_Sensors range_sensors;

        float front_distance;
        float rear_distance;
        float delta_theta;
        float delta_d;
        float delta_speed;
        float sensor_gap = 100.0;
        float platform_distance = 45.0;
        float turning_offset_speed = 32.0;
        float k_d = .25;
        float k_theta = 10.0;
  
    public:

        bool Initialize(unsigned char in_speed);

        void SetSpeed(unsigned char in_speed);
        void DriveForward();
        void DriveForwardWithSteering();
        void TurnIntoRow();
        void DriveBackward();
        void TurnCounterClockwise();
        void TurnClockwise();
        void ExtendMast();
        void RetractMast();
        void Stop();
};

#endif
