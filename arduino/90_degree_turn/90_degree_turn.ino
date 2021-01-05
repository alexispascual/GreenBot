#include <Arduino.h>

unsigned char hero_message[3];
unsigned char turning_speed = 42;
unsigned char message_length = 3;

void setup() {

	hero_message[0] = 0x80;

	Serial1.begin(115200);
  	delay(100);
  	Serial1.println("Start");
  
}

void loop() {

	hero_message[1] = turning_speed;
	hero_message[2] = turning_speed + 0x80;

	Serial1.write(hero_message, message_length);

	delay(5000);

  	hero_message[1] = 0x00;
	hero_message[2] = 0x00;

	Serial1.write(hero_message, message_length);

	delay(5000);

}
