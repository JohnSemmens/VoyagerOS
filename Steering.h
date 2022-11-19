// Steering.h

#ifndef _STEERING_h
#define _STEERING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


void SteeringFastUpdate(void);
void SteeringPID_Init(void);

#endif

