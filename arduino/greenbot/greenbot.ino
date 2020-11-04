bool new_data = false;
const byte num_chars = 32;

float cmd_vel[num_chars];
char raw_chars[num_chars];

const byte PWM_pin_3 = 3; // front left
const byte PWM_pin_5 = 5; // front right
const byte PWM_pin_6 = 6; // rear left
const byte PWM_pin_9 = 9; // rear right
const byte PWM_pin_10 = 10; // mast

void setup() {

  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

  pinMode(PWM_pin_3, OUTPUT);
  pinMode(PWM_pin_5, OUTPUT);
  pinMode(PWM_pin_6, OUTPUT);
  pinMode(PWM_pin_9, OUTPUT);
  pinMode(PWM_pin_10, OUTPUT);
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

    // Drive Forward or Backwards
    if (cmd_vel[1] == 0) {

      if (cmd_vel[0] > 0) { // Forward
        Serial.println("Moving forward \n");
        analogWrite(PWM_pin_3, cmd_vel[0]);
        analogWrite(PWM_pin_5, cmd_vel[0]);
        analogWrite(PWM_pin_6, cmd_vel[0]);
        analogWrite(PWM_pin_9, cmd_vel[0]);
      } else { // TODO: Reverse
        Serial.println("Moving backward \n");
        analogWrite(PWM_pin_3, cmd_vel[0]);
        analogWrite(PWM_pin_5, cmd_vel[0]);
        analogWrite(PWM_pin_6, cmd_vel[0]);
        analogWrite(PWM_pin_9, cmd_vel[0]);
      }

    }
    // Turn without moving forwards or backwards
    else if (cmd_vel[0] == 0) {

        if (cmd_vel[1] > 0) {// Turn counter-clockwise
          Serial.println("Turning counter-clockwise \n");
          analogWrite(PWM_pin_5, cmd_vel[1]);
          analogWrite(PWM_pin_9, cmd_vel[1]);
        } else { // Turn Clockwise
          Serial.println("Turning clockwise \n");
          analogWrite(PWM_pin_3, cmd_vel[1]);
          analogWrite(PWM_pin_6, cmd_vel[1]);
        }
    }

    delay(10); // Move for a few milliseconds before waiting for a new command.
    analogWrite(PWM_pin_3, 0);
    analogWrite(PWM_pin_5, 0);
    analogWrite(PWM_pin_6, 0);
    analogWrite(PWM_pin_9, 0);
    new_data = false;
  }
}

void loop() {

  ReceiveSerialData();
  ParseRawChars();
  HandleCmdVelData(cmd_vel);
  delay(10);

}
