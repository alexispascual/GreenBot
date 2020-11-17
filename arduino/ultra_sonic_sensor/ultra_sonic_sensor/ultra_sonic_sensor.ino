

#define NUM_SENSORS 4

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

uint8_t trigger_pins[NUM_SENSORS] = {0, 2, 4, 6};
uint8_t echo_pins[NUM_SENSORS] = {1, 3, 5, 7};

void setup() {

  for (byte i = 0; i < NUM_SENSORS; i++) {
    
    pinMode(trigger_pins[i], OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echo_pins[i], INPUT); // Sets the echoPin as an INPUT
    
  }
  
  Serial.begin(11520); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor

}
void loop() {

  for (byte i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(trigger_pins[i], LOW); // Clears the trigPin condition
  }

  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds

  for (byte i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(trigger_pins[i], HIGH); // Set trigger pin high for 10 micro seconds
  }
  
  delayMicroseconds(10);

  for (byte i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(trigger_pins[i], LOW); // Clear the trigger pin
  }

  for (byte i = 0; i < NUM_SENSORS; i++) {
    duration = pulseIn(echo_pins[i], HIGH);// Reads the echoPin, returns the sound wave travel time in microseconds
  }
  
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  
  Serial.print("Distance: ");
  Serial.print(distance); 
  Serial.println(" cm");
  delay(1000);
}
