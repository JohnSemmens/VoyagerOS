// PowerManagement.h

#ifndef _POWERMANAGEMENT_h
#define _POWERMANAGEMENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void InitPowerManagement(void);

void TelemetryPowerOn(boolean PowerOn);

void RCPowerOn(boolean PowerOn);

void PowerControl(int controlMask);

#endif

