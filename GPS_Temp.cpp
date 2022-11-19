// Wrapper for the TinyGPS - Temp wrapper for I2C / serial GPS
// This provides a minimalist implementation of a GPS.
// It uses the I2C bus 
// V1.0 19/7/2016 Created
// V1.1 22/10/2016 updated to support parameterised serial port
// V1.2 updated 21/10/2018 changed from Serial to I2C
// V1.3 28/10/2018 added support for converting from CurrentUTCTime provided by the GPS to CurrentLocalTime

#include "GPS_Temp.h"
#include "GPS.h"
#include "Navigation.h"
#include <Wire.h>
#include "sim_vessel.h"

extern NavigationDataType NavData;
extern long GPS_Last_Loc;						// milliseconds since last GPS valid location message
extern long GPS_Last_Message;					// milliseconds since last GPS message
extern Time CurrentUTCTime;
extern Time CurrentLocalTime;
extern bool UseSimulatedVessel;			// flag to disable the GPS and indicate that the current location is simulated 
extern sim_vessel simulated_vessel;

void GPS_Temp_Read() {
	// V1.1 27/4/2019 Temp GPS reader for Temp I2C Arduino interface to a serial GPS
	// V1.2 10/1/2021 updated to continue using GPS for Time Stamp while running in Simulated position mode.
	//		previously we had no time on the logs in this case.

	static long prev_GPS_Last_Message;
	static long prev_GPS_Last_Loc;
	
	Wire.requestFrom(0x29, 18);       // Ask for 18 bytes

	long Lat=0, Long=0;					// 8 bytes
	uint8_t Year =0, Month=0, Day=0;    // 3 bytes
	uint8_t Hour=0, Minute=0, Second=0; // 3 bytes
	int COG=0, SOG=0;					// 8 bytes

	for (int i = 0; i<4; i++) 
		Lat = Lat | (long)Wire.read() << (i * 8);

	for (int i = 0; i<4; i++)
		Long = Long | (long)Wire.read() << (i * 8);

	Year = Wire.read();
	Month = Wire.read();
	Day = Wire.read();
	Hour = Wire.read();
	Minute = Wire.read();
	Second = Wire.read();

	for (int i = 0; i<2; i++)
		COG = COG | Wire.read() << (i * 8);

	for (int i = 0; i<2; i++)
		SOG = SOG | Wire.read() << (i * 8);

	// get the date and time from GPS into CurrentTime object.
	CurrentUTCTime.year = Year;
	CurrentUTCTime.month = Month;
	CurrentUTCTime.dayOfMonth = Day;
	CurrentUTCTime.hour = Hour;
	CurrentUTCTime.minute = Minute;
	CurrentUTCTime.second = Second;


	// if not using a simulated GPS position (i.e. if real) then populate the NavData
	if (!UseSimulatedVessel)
	{
		NavData.Currentloc.lat = Lat; 
		NavData.Currentloc.lng = Long; 
		NavData.CurrentLocTimeStamp = millis();

		// get course and speed directly from GPS
		NavData.COG = ((float)COG/100);
		NavData.SOG_mps = ((float)SOG / 100);

		// update gps message timing statistics
		GPS_Last_Message = millis() - prev_GPS_Last_Message;
		prev_GPS_Last_Message = millis();

		// update gps valid location timing statistics
		if (NavData.Currentloc.lat != 0 && NavData.Currentloc.lng != 0) {
			GPS_Last_Loc = millis() - prev_GPS_Last_Loc;
			prev_GPS_Last_Loc = millis();
		}	
	}
	else // populate with simulated data
	{
		NavData.Currentloc.lat = simulated_vessel.Currentloc.lat;
		NavData.Currentloc.lng = simulated_vessel.Currentloc.lng;
		NavData.CurrentLocTimeStamp = millis();

		NavData.COG = simulated_vessel.Heading;
	    NavData.SOG_mps = simulated_vessel.SOG_mps;
	}
};






