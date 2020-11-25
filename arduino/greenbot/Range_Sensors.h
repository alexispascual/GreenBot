#ifndef Range_Sensors_h
#define Range_Sensors_h

//--------------------------------------------------------------------------//
//                                  Imports                                 //
//--------------------------------------------------------------------------//

#include "Arduino.h"

class Range_Sensors {
//--------------------------------------------------------------------------//
//                                  Definitions                             //
//--------------------------------------------------------------------------//
	private:

		#define NUM_SENSORS 4

		uint8_t trigger_pins[NUM_SENSORS] = {0, 2};
		uint8_t echo_pins[NUM_SENSORS] = {1, 3};

		float front_distance;
		float rear_distance;

		void Activate_Sensors();
		
	public:
		Range_Sensors(uint8_t* trigger_pins, uint8_t* echo_pins);
		float Get_Front_Distance();
		float Get_Rear_Distance();
		float* Get_Distances();
}

#endif
