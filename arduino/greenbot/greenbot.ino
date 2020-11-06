#include "Greenbot.h"
#include <Servo.h>

#define PWM_HIGH_DUR 1600
#define PWM_NEUTRAL_DUR 1500
#define PWM_LOW_DUR 1400

bool new_data = false;
const byte num_chars = 32;

float cmd_vel[num_chars];
char raw_chars[num_chars];

const byte PWM_pin_10 = 10; // right_wheels
const byte PWM_pin_11 = 11; // left_wheels

Servo left_wheels;
Servo right_wheels;

void setup() {

  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

  right_wheels.detach();
  left_wheels.detach();

  right_wheels.attach(PWM_pin_10);
  left_wheels.attach(PWM_pin_11);
}


void ReceiveSerialData() {
    static boolean in_progress = false;
    static byte i = 0;
    char start_marker = '[';
    char end_marker = ']';
    char temp_char;
    float temp_float;
 
    while (Serial.available() > 0 && new_data == false) {
        temp_char = Serial.read();

        if (in_progress == true) {
            if (temp_char != end_marker) {
              raw_chars[i] = temp_char;
                i++;
                if (i >= num_chars) {
                    i = num_chars - 1;
                }
            }
            else {
                cmd_vel[i] = '\0'; // terminate the string
                in_progress = false;
                i = 0;
                new_data = true;
            }
        }

        else if (temp_char == start_marker) {
            in_progress = true;
        }
    }
}

void ParseRawChars() {
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(raw_chars,",");      // get the first part - the string
  cmd_vel[0] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  cmd_vel[1] = atof(strtokIndx);     // convert this part to a float
    
}

void HandleCmdVelData(float cmd_vel[]) {
  
  if (new_data) {

    int pwm_signal_1;
    int pwm_signal_2;


    if (cmd_vel[0] > 0 && cmd_vel[1] == 0) { // Forward

      Serial.println("Moving forward \n");

      right_wheels.writeMicroseconds(PWM_HIGH_DUR);
      left_wheels.writeMicroseconds(PWM_HIGH_DUR);

    } else if (cmd_vel[0] < 0 && cmd_vel[1] == 0) { // TODO: Reverse

      Serial.println("Moving backward \n");

      right_wheels.writeMicroseconds(PWM_LOW_DUR);
      left_wheels.writeMicroseconds(PWM_LOW_DUR);
    }

    else if (cmd_vel[0] == 0 && cmd_vel[1] > 0) { // Turn counter-clockwise
      
      Serial.println("Turning counter-clockwise \n");

      right_wheels.writeMicroseconds(PWM_HIGH_DUR);
      left_wheels.writeMicroseconds(PWM_LOW_DUR);
    }

    else if (cmd_vel[0] == 0 && cmd_vel[1] < 0) { // Turn Clockwise

      Serial.println("Turning clockwise \n");

      right_wheels.writeMicroseconds(PWM_LOW_DUR);
      left_wheels.writeMicroseconds(PWM_HIGH_DUR);
    }

    else if (cmd_vel[0] == 0 && cmd_vel[1] == 0) { // Stop

      Serial.println("Stopping \n");

      right_wheels.writeMicroseconds(PWM_NEUTRAL_DUR);
      left_wheels.writeMicroseconds(PWM_NEUTRAL_DUR);
      
    }
    
    new_data = false;
  }

}

int ScaleVelocity(float vel) {

  float scaled_vel = 0;
 
  scaled_vel = (vel + 255) *(2400/510);
  Serial.println(scaled_vel);
  return (int)scaled_vel;

}

void loop() {

  ReceiveSerialData();
  ParseRawChars();
  HandleCmdVelData(cmd_vel);
  delay(10);

}
