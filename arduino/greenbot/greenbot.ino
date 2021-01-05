
#ifndef Greenbot_h
  #include "Greenbot.h"
#endif

#ifndef Range_Sensors_h
  #include "Range_Sensors.h"
#endif
  
#include "Command.h"
#include <Servo.h>

const uint8_t num_chars = 32;
const uint8_t default_speed = 16; //default speed

bool new_data = false;
bool greenbot_status = false;
char raw_chars[num_chars];
int8_t current_speed;

Command command;
Greenbot greenbot;
Range_Sensors range_sensors;

void setup() {

  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

  Serial1.begin(115200);
  delay(100);
  Serial1.println("Start");

  current_speed = default_speed;
  
  // Initialize Greenbot object  
  greenbot_status = greenbot.Initialize(default_speed);

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

  strtokIndx = strtok(NULL, ","); // Get speed
  command.speed = atoi(strtokIndx);
    
}

void HandleCommand() {
  
  if (new_data) {

    new_data = false;
    
    if (command.speed != current_speed) {

      greenbot.SetSpeed((unsigned char)command.speed);

      current_speed = command.speed;
    }

    if (command.x == 1) { // Forward

      greenbot.DriveForward();
      
    } else if (command.x == -1) { // Reverse

      greenbot.DriveBackward();
      
    } else if (command.z > 0) { // Turn counter-clockwise
      
      greenbot.TurnCounterClockwise();
      
    } else if (command.z < 0) { // Turn Clockwise

      greenbot.TurnClockwise();

    } else if (command.mast_control > 0) { // Extend mast
      
      greenbot.ExtendMast();
      
    } else if (command.mast_control < 0) { // Retract mast
      
      greenbot.RetractMast();
      
    } else if (command.x == 0 && command.z == 0 && command.mast_control == 0) { // Stop

      greenbot.Stop();
      
    } else if (command.x == 2) {
      
      while (!new_data){

          greenbot.DriveForwardWithSteering();
          ReceiveSerialData();

        }
        
    } else if (command.x == 3) { // Turn into new row **DEPRECATED**
      
      greenbot.TurnIntoRow();
      
    } else if (command.x == 4) { // Execute distance correction
      
      greenbot.ExecuteDistanceCorrection();
      
    } else if (command.x == 5) { // Print attitude and distance
      
      range_sensors.GetAttitude();
      range_sensors.GetRoverDistance();
    }


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
