// RadioControl.h

#ifndef _RADIOCONTROL_h
#define _RADIOCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct RC_IN_Type {
	uint16_t Channel[4];    // pulse width in microseconds, array for each of the input channels
	bool SignalValid;		// flag to indicate if the Radio Control is operational. 
	long LastRCSignalTime;	// time in millis when the last RC signal was received. This is used to establish whether the RC is functional
	int RC_Command_Switch_Position; // this represents integer values of the "Virtual" multi position switch derived from an RC channel
};

void RC_Init(void);
void RC_Read(void);
void RC_Command_Switch(void);

void Servos_Init(void);
void Servo_Out(int Channel, int PulseWidth);

#endif
