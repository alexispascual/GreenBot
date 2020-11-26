#include <Servo.h>
#include <Arduino.h>

Servo right_wheels;
Servo left_wheels;

float frequency = 0.01;
float amplitude = 200;
float test_duration = 10000;

float right_pulse_width;
float left_pulse_width;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Start");
  
	right_wheels.attach(52);
	left_wheels.attach(53);

}

void loop() {
  // put your main code here, to run repeatedly:
	for (int i = 0; i <= test_duration; i++) {

		right_pulse_width = 1550 + amplitude*sin(TWO_PI*frequency*i);
		left_pulse_width = 1550 - amplitude*sin(TWO_PI*frequency*i);
    delay(100);

    if (right_pulse_width < 0) {
      right_pulse_width = 0;
      }
    else if (left_pulse_width < 0) {
      left_pulse_width = 0;
      }

    right_wheels.writeMicroseconds(right_pulse_width);
    left_wheels.writeMicroseconds(left_pulse_width);
    
    Serial.println(right_pulse_width);
    Serial.println(left_pulse_width);
	}

}
