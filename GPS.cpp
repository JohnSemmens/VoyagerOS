// Wrapper for the TinyGPS
// This provides a minimalist implementation of a GPS.
// It uses the I2C bus 
// V1.0 19/7/2016 Created
// V1.1 22/10/2016 updated to support parameterised serial port
// V1.2 updated 21/10/2018 changed from Serial to I2C
// V1.3 28/10/2018 added support for converting from CurrentUTCTime provided by the GPS to CurrentLocalTime

#include "GPS.h"
#include "Navigation.h"
#include "location.h"
#include "configValues.h"
#include "SparkFun_I2C_GPS_Arduino_Library.h" 
#include "TinyGPS++.h"

extern TinyGPSPlus gps;
extern NavigationDataType NavData;
extern HardwareSerial *Serials[];
extern configValuesType Configuration;
extern I2CGPS myI2CGPS;

long GPS_Last_Loc;						// milliseconds since last GPS valid location message
long GPS_Last_Message;					// milliseconds since last GPS message

extern bool UseSimulatedVessel;			// flag to disable the GPS and indicate that the current location is simulated 
extern Time CurrentUTCTime;
extern Time CurrentLocalTime;

void GPS_Init(int CommandPort) {
	// Initialise the GPS 
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 21/10/2018 updated for change from serial to I2C 
	// V1.3 6/4/2019 added delay at the end, because it seems solve a lock up problem.

	(*Serials[CommandPort]).println(F("MSG,Checking for GPS at 0x10 ..."));

	if (myI2CGPS.begin() == false)
	{
		(*Serials[CommandPort]).println(F("MSG,No GPS detected at 0x10."));
	}
	else
	{
		(*Serials[CommandPort]).println(F("MSG,GPS detected at 0x10."));
	}

	GPS_Read_Time();
	UpdateLocalTimeFromGPSTime(Configuration.timezone_offset);
	GPS_Read();
	delay(300); // it seems to be necessary to delay a bit before doing anything else or a lock up occurs.
				// not sure what's happening, but this seems to fix it.
				// was 100ms. changed to 300ms because of some occasional start up lockups to see if it helps.
};

void GPS_Read() {
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 14/4/2018 added GPS directly provided course and speed to Navdata object
	// V1.3 21/10/2018 updated for change from serial to I2C 

	static long prev_GPS_Last_Message;
	static long prev_GPS_Last_Loc;

	// Read the GPS and place the result into the Global variable: Currentloc
	while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
	{
		gps.encode(myI2CGPS.read()); //Feed the GPS parser
	}

	NavData.Currentloc.lat = gps.location.lat() * 10000000UL;
	NavData.Currentloc.lng = gps.location.lng() * 10000000UL;
	NavData.CurrentLocTimeStamp = millis();

	// get course and speed directly from GPS
	NavData.COG = gps.course.deg();

	NavData.SOG_knt = (float)gps.speed.knots();
	NavData.SOG_mps = (float)gps.speed.mps();

	// get the date and time from GP into CurrentTime object.
	CurrentUTCTime.year = gps.date.year()-2000;
	CurrentUTCTime.month = gps.date.month();
	CurrentUTCTime.dayOfMonth = gps.date.day();

	CurrentUTCTime.hour = gps.time.hour();
	CurrentUTCTime.minute = gps.time.minute();
	CurrentUTCTime.second = gps.time.second();


	// update gps message timing statistics
	GPS_Last_Message = millis() - prev_GPS_Last_Message;
	prev_GPS_Last_Message = millis();

	// update gps valid location timing statistics
	if (NavData.Currentloc.lat != 0 && NavData.Currentloc.lng != 0) {
		GPS_Last_Loc = millis() - prev_GPS_Last_Loc;
		prev_GPS_Last_Loc = millis();
	}	
	
};


void GPS_Read_Time() {
	// V1.0 21/10/2018 Read the Date and Time from the GPS
	// V1.1 30/3/2019 updated to return a reasonably formatted date, even if the GPS Time is not available.
	//				This is to make the system more resilient. 

	while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
	{
		gps.encode(myI2CGPS.read()); //Feed the GPS parser
	}

	// get the date and time from GP into CurrentTime object.
	CurrentUTCTime.year = gps.date.year() - 2000;
	CurrentUTCTime.month = gps.date.month();
	CurrentUTCTime.dayOfMonth = gps.date.day();

	CurrentUTCTime.hour = gps.time.hour();
	CurrentUTCTime.minute = gps.time.minute();
	CurrentUTCTime.second = gps.time.second();

	if (CurrentUTCTime.year == 0)
	{
		CurrentUTCTime.year = 99; // the year
		CurrentUTCTime.month = 9;
		CurrentUTCTime.dayOfMonth = 9;
		CurrentUTCTime.hour = 99;
		CurrentUTCTime.minute = 99;
	}
};

void UpdateLocalTimeFromGPSTime(byte timezone_offset)
{
	// update the CurrentLocalTime from the CurrentUTCTime provided from the GPS sentences.
	// use the offset passed in as a parameter.

	CurrentLocalTime.second = CurrentUTCTime.second;
	CurrentLocalTime.minute = CurrentUTCTime.minute;

	CurrentLocalTime.hour = CurrentUTCTime.hour + timezone_offset;

	// if we've moved into next day, then increment day, and deduct 24 hours from hour. 
	if (CurrentLocalTime.hour >= 24)
	{
		CurrentLocalTime.dayOfMonth = CurrentUTCTime.dayOfMonth + 1;
		CurrentLocalTime.hour = CurrentLocalTime.hour - 24;
	}
	else
	{
		CurrentLocalTime.dayOfMonth = CurrentUTCTime.dayOfMonth;
	}

	// if we've rolled past end of month, then increment month and reset day number
	if (CurrentLocalTime.dayOfMonth > DaysInMonth(CurrentLocalTime.month, CurrentLocalTime.year))
	{
		CurrentLocalTime.month = CurrentUTCTime.month + 1;
		CurrentLocalTime.dayOfMonth = 1;
	}
	else
	{
		CurrentLocalTime.month = CurrentUTCTime.month;
	}

	// if we've rolled into month 13 then increment year and reset month.
	if (CurrentLocalTime.month >= 13)
	{
		CurrentLocalTime.year = CurrentUTCTime.year + 1;
		CurrentLocalTime.month = 1;
	}
	else
	{
		CurrentLocalTime.year = CurrentUTCTime.year;
	}
}


byte DaysInMonth(byte month, byte year)
{
	// return the number of days on a given month.
	// this is used in converting from between time zones.
	// The test for leap years has bee simplified, because the next issue other than usual 4 year test, is in the year 2100.

	// https://www.daniweb.com/programming/software-development/threads/265791/calculating-days-in-a-month

	int Days;
	if (month == 4 || month == 6 || month == 9 || month == 11)
		Days = 30;

	else if (month == 02)
	{
		//bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
		// use a simpler/faster leap year check. It will be 80 years before it matters.
		bool isLeapYear = (year % 4 == 0);

		if (isLeapYear == 0)
			Days = 29;
		else
			Days = 28;
	}
	else
		Days = 31;

	return Days;
}

bool GPS_LocationIs_Valid(Location TestLoc) {
	// check if the GPS location is valid (or if the simulated location is valid)
	// V1.1 21/10/2018 Updated to allow GPS_Last_Loc time in ms of zero to considered ok (i.e ">=" vs ">")

	bool valid = false;

	if (UseSimulatedVessel)
	{
	  // if simulated, just check for non-zero lat/lon 
	  valid = (TestLoc.lat != 0 && TestLoc.lng != 0);
	}
	else
	{
	  // if real GPS, check for non=zero lat/lon and valid location provided in last 10 seconds
	  valid = (TestLoc.lat != 0 && TestLoc.lng != 0 && GPS_Last_Loc >= 0 && GPS_Last_Loc < 10000);
	}

	return valid;
};

bool GPS_TimeIs_Valid() {
	// check if the GPS time is valid 
	// V1.1 19/05/2020 John Semmens

	return (CurrentUTCTime.month > 0);
};