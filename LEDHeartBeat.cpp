// 
// 
// 

#include "LEDHeartBeat.h"



void LED_HeartBeat(int pin)
// toggle the LED
{
	static bool ledstate;
	pinMode(pin, OUTPUT);

	if (ledstate == true) {
		ledstate = false;
		digitalWrite(pin, HIGH);
	}
	else {
		ledstate = true;
		digitalWrite(pin, LOW);
	}
}