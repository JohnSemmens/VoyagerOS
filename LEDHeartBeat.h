// LEDHeartBeat.h

#ifndef _LEDHEARTBEAT_h
#define _LEDHEARTBEAT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void LED_HeartBeat(int pin);

#endif

