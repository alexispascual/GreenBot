bool new_data = false;
const byte num_chars = 32;

float cmd_vel[num_chars];
char raw_chars[num_chars];

void setup() {

  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");
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
    Serial.print("Received the following:");
    Serial.print(cmd_vel[0]);
    Serial.print("\n");
    Serial.print(cmd_vel[1]);
    Serial.print("\n");

    new_data = false;
    }
  
}

void loop() {

  ReceiveSerialData();
  ParseRawChars();
  HandleCmdVelData(cmd_vel);
  delay(10);

}
