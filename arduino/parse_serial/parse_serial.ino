

const uint8_t num_chars = 32;
const uint8_t default_speed = 127; //default speed
int8_t left_wheel_speed;
int8_t right_wheel_speed;
bool new_data = false;
char raw_chars[num_chars];

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

}

void ReceiveSerialData() {

    static boolean in_progress = false;
    static byte i = 0;
    char start_marker = 0x80;
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
  
  left_wheel_speed = raw_chars[0]; //Get left wheel speed

  right_wheel_speed =  raw_chars[1]; // Get right wheel speed
    
}

void PrintData() {
  
  if (new_data) {
    
    new_data = false;
    
    Serial.print("Left wheel: ");
    Serial.print(left_wheel_speed);
    Serial.print("Right wheel: ");
    Serial.print(right_wheel_speed);
    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  ReceiveSerialData();
  ParseRawChars();
  PrintData();
  
}
