// 
// 
// 

#include "PowerManagement.h"

extern byte OLED_PowerPin; 
extern byte GPS_PowerPin;

extern byte RC_PowerPin;
extern byte Telemetry_PowerPin;


void InitPowerManagement(void)
{
	// these two don't work properly, so just turn them on 
	pinMode(OLED_PowerPin, OUTPUT); // set the power control pin for the i2c connector OLED display to output
	digitalWrite(OLED_PowerPin, LOW); // turn on the Display

	pinMode(GPS_PowerPin, OUTPUT); // set the power control pin for the GPS i2c connector to output
	digitalWrite(GPS_PowerPin, LOW); // turn on the GPS i2c connector 


	// turn on RC
	pinMode(RC_PowerPin, OUTPUT); // set the power control pin for the RC to output
	RCPowerOn(false);  // turn off the RC

	// turn on Telemetry
	pinMode(Telemetry_PowerPin, OUTPUT); // set the power control pin for the TEL to output
	TelemetryPowerOn(false); // turn off the TEL
};


void TelemetryPowerOn(boolean PowerOn)
{
	digitalWrite(Telemetry_PowerPin, !PowerOn);
};

void RCPowerOn(boolean PowerOn)
{
	digitalWrite(RC_PowerPin, !PowerOn);
};

void PowerControl(int controlMask)
{
	// power controller
	// V1.1 10/1/2021 force Telemetry to be always on.
	
	// Bit 0
	RCPowerOn(((controlMask & 1) == 1));

	// Bit 1
	// force the telemtry power to be always on.
	// there's no need to power down, because is only transmits when requested. 
	TelemetryPowerOn(true);
	//TelemetryPowerOn(((controlMask & 2) == 2));
};