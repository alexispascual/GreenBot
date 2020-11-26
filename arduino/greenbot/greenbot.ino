
#ifndef Greenbot_h
  #include "Greenbot.h"
#endif

#ifndef Range_Sensors_h
  #include "Range_Sensors.h"
#endif
  
#include "Command.h"
#include <Servo.h>

const uint8_t num_chars = 32;
const uint8_t pwm_pins_mast = 9; //mast
const uint8_t pwm_pins_right = 10; // right_wheels
const uint8_t pwm_pins_left = 11; // left_wheels
const uint8_t default_speed = 127; //default speed

Servo left_wheels;
Servo right_wheels;

bool new_data = false;
bool greenbot_status = false;
char raw_chars[num_chars];
uint8_t current_speed;

Command command;
Greenbot greenbot;

void setup() {

  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

  current_speed = default_speed;
  
  // Initialize Greenbot object  
  greenbot_status = greenbot.Initialize(pwm_pins_right, pwm_pins_left, pwm_pins_mast, default_speed);

  if (greenbot_status) {

    Serial.println("Greenbot successfully initialized!");

  } else {

    Serial.println("Greenbot initialization failed!");

  }

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
                raw_chars[i] = '\0'; // terminate the string
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

  strtokIndx = strtok(raw_chars,","); //Get x
  command.x = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  command.z = atoi(strtokIndx);     // Get z

  strtokIndx = strtok(NULL, ",");
  command.mast_control = atoi(strtokIndx);  // Get mast_control

  strtokIndx = strtok(NULL, ",");
  command.speed = atof(strtokIndx);
    
}

void HandleCommand() {
  
  if (new_data) {

    if (command.x > 0) { // Forward

      Serial.println("Moving forward \n");
      greenbot.DriveForward();
      
    } else if (command.x < 0) { // Reverse

      Serial.println("Moving backward \n");
      greenbot.DriveBackward();
      
    } else if (command.z > 0) { // Turn counter-clockwise
      
      Serial.println("Turning counter-clockwise \n");
      greenbot.TurnCounterClockwise();
      
    } else if (command.z < 0) { // Turn Clockwise

      Serial.println("Turning clockwise \n");
      greenbot.TurnClockwise();

    } else if (command.mast_control > 0) { // Extend mast
      
      Serial.println("Extending mast \n");
      greenbot.ExtendMast();
      
    } else if (command.mast_control < 0) { // Retract mast
      
      Serial.println("Retracting mast \n");
      greenbot.RetractMast();
      
    } else if (command.x == 0 && command.z == 0 && command.mast_control == 0) { // Stop

      Serial.println("Stopping \n");
      greenbot.Stop();
      
    }

    if (command.speed != current_speed) {

      Serial.println("Updating Greenbot Speed");
      greenbot.SetSpeed((uint8_t)command.speed);

      current_speed = (uint8_t)command.speed;
      
    }
    
    new_data = false;
  }

}

void loop() {

  if (!greenbot_status) {

    while(1){
      Serial.println("Greenbot initialization failed!");
      delay(1000);
    }
  } else {
    ReceiveSerialData();
    ParseRawChars();
    HandleCommand();
    delay(10);
  }



}
