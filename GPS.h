// Wrapper for the TinyGPS
// This provides a minimalist implementation of a GPS.
// It uses the I2C bus .
// V1.0 Created 19/7/2016
// V1.1 updated 21/10/2018 changed from Serial to I2C

#ifndef _GPS_h
#define _GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Waypoints.h"

struct Time {
	byte second;
	byte minute;
	byte hour;
	byte dayOfWeek;
	byte dayOfMonth;
	byte month;
	byte year;
	int raw;
};

// initialise the software serial port for the GPS.
void GPS_Init(int CommandPort);

// Read the GPS and place the result into the Global variable: Currentloc
void GPS_Read(void);

// check if the GPS location is valid (or if the simulated location is valid)
bool GPS_LocationIs_Valid(Location TestLoc);

bool GPS_TimeIs_Valid();

void GPS_Read_Time(void);
void UpdateLocalTimeFromGPSTime(byte timezone_offset);
byte DaysInMonth(byte month, byte year);


#endif

