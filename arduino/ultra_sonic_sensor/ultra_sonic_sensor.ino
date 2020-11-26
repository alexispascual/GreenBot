#ifndef Range_Sensors_h
  #include "Range_Sensors.h"
#endif

// define variables

float front_distance; //
float rear_distance; // 

uint8_t trigger_pins[NUM_SENSORS] = {0, 2};
uint8_t echo_pins[NUM_SENSORS] = {1, 3};

Range_Sensors range_sensors;

void setup() {
  
  // initialize serial port at a baud rate of 115200 bps
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");

  range_sensors.Initialize(trigger_pins, echo_pins, 2);

}
void loop() {

  front_distance = range_sensors.Get_Front_Distance();
  rear_distance = range_sensors.Get_Rear_Distance();
  
  Serial.print("Front: ");
  Serial.print(front_distance); 
  Serial.println(" cm");

  Serial.print("Rear: ");
  Serial.print(rear_distance); 
  Serial.println(" cm");
  delay(1000);
}
