
// SD Card Logging 
// 
// V1.0 3/8/2016 John Semmens
// V1.8 29/10/2017 updated for new IMU BNO-055 rather than MPU-9250.
// V1.9 12/11/2017 updated to change BRL to RLB.
// V1.10 15/4/2018 Added Servo output logging SVO and Voltage montitoring VLT.
// V1.11 17/4/2018 bug fix Servo output logging SVO and Voltage montitoring VLT.
// V1.12 28/10/2018 changed to CurrentLocalTime
// V1.13 25/4/2019 extended SAI to include Turn Heading and Target Heading, and added Temp and Press to System.
// V1/14 11/6/2019 added Trim tab Angle to SVO sentence, and Wing Angle to ENV. and IsBTWSailable to NAV
// V1.15 9/7/2019 added Decision Event Value to logging.
// V1.16 11/7/2019 added Heading Error from GPS

#include "SDCardLogFile.h"
#include <SPI.h>
#include "SdFat.h"
#include "GPS.h"
#include "Navigation.h"
#include "USFS_IMU.h"
#include "Mission.h"
#include "configValues.h"
#include "CommandState_Processor.h"
#include "BluetoothConnection.h"
#include "WearTracking.h"
#include "Wingsail.h"
#include "Mission.h"

#include "OLED_Logging.h"

extern SdFat sd;
extern SdFile LogFile;
extern int CommandPort;

#define  SD_Card_CS_PIN 53 // 10 on Nano, 53 on Mega, E3 on Teensy 3.6   

extern StateValuesStruct StateValues;
extern NavigationDataType NavData;
extern IMUStruct myIMU;
extern long loop_period_us; // microseconds between successive main loop executions
extern long GPS_Last_Loc;		// milliseconds since last GPS valid location message
extern long GPS_Last_Message;	// milliseconds since last GPS message
extern bool UseSimulatedVessel; 	// flag to disable the GPS and indicate that the current location is simulated 
extern configValuesType Configuration;
extern HardwareSerial *Serials[];
extern bool SD_Card_Present;
extern Time CurrentLocalTime;

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int DecisionEventValue;							// this a value relevant to an event
extern int DecisionEventValue2;							// this a value relevant to an event
extern VesselUsageCountersStruct VesselUsageCounters;

extern double SteeringServoOutput;

extern double PowerSensorV, PowerSensorI;
extern BTStateType BTState;
extern WingSailType WingSail;
extern MissionValuesStruct MissionValues;
extern char Version[];

void dateTime(uint16_t* date, uint16_t* time)
{
	// V1.0 1/10/2016 John Semmens
	unsigned int year = int(2000 + CurrentLocalTime.year);
	byte month = CurrentLocalTime.month;
	byte day = CurrentLocalTime.dayOfMonth;
	byte hour = CurrentLocalTime.hour;
	byte minute = CurrentLocalTime.minute;
	byte second = CurrentLocalTime.second;

	// YOUR SKETCH SHOULD UPDATE THESE SIX Values
	// EACH TIME BEFORE CREATING YOUR SD CARD FILE
	*date = FAT_DATE(year, month, day);
	*time = FAT_TIME(hour, minute, second);
}

void LogTimeHeader(void)
{
	// V1.0 1/10/2016 John Semmens
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("YYYY"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("MM"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("DD"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("HH"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("mm"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("ss"));
	LogFile.print(Configuration.SDCardLogDelimiter);
}

void SD_Logging_Init(int CommandPort) {
	// V1.0 1/10/2016 John Semmens
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 3/11/2016 added flag for the presence of the SD Card

	(*Serials[CommandPort]).print(F("MSG,Initializing SD card..."));

	// THIS LINE SETS YOUR SKETCH TO SAVE YOUR
	// TIME AND DATE INTO ANY FILE YOU CREATE.
	SdFile::dateTimeCallback(dateTime);

	if (!sd.begin(SD_Card_CS_PIN)) {
		(*Serials[CommandPort]).println(F("Initialization failed. Card may not be present."));
		SD_Card_Present = false;
	}
	else {
		(*Serials[CommandPort]).println(F("Initialization done."));
		SD_Card_Present = true;
	}
}

void SD_Logging_OpenFile(int CommandPort,word LoggingMask){
	// V1.0 10/10/2016 John Semmens
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 30/10/2017 added IMU status to Attitude

	// set up 8.3 filename

	//String year = "20" + String(CurrentTime.year);
	String month = (CurrentLocalTime.month < 10 ? "0": "") + String(CurrentLocalTime.month);
	String day = (CurrentLocalTime.dayOfMonth < 10 ? "0" : "") + String(CurrentLocalTime.dayOfMonth);
	String hour = (CurrentLocalTime.hour < 10 ? "0" : "") + String(CurrentLocalTime.hour);
	String minute = (CurrentLocalTime.minute < 10 ? "0" : "") + String(CurrentLocalTime.minute);
	//String second = (CurrentLocalTime.second < 10 ? "0" : "") + String(CurrentLocalTime.second);
	String LogFileName =  month + day + hour + minute +".log";

	(*Serials[CommandPort]).print(F("MSG,Open SD Card Logfile:"));
	(*Serials[CommandPort]).println(LogFileName);
	OpenSDLogFile(LogFileName);
	delay(30);

	(*Serials[CommandPort]).print(F("MSG,SD Card Logging Mask:"));
	(*Serials[CommandPort]).println(Configuration.SDCardLoggingMask);

	// Log_Location  loc - Time, Lat, Lon
	LogFile.print(F("Data"));
	LogTimeHeader();
	LogFile.print(F("Field1"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field2"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field3"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field4"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field5"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field6"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field7"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field8"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field9"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("Field10"));
	LogFile.println();

	// add sd card logging Header based on selected mask.
	if (Log_Location & LoggingMask)
	{
		// Log_Location  loc - Time, Lat, Lon
		LogFile.print(F("LOC"));
		LogTimeHeader();
		LogFile.print(F("LAT"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("LON"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("COG"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("SOG")); //   m/s
	}

	if (Log_Attitude & LoggingMask)
	{
		// Log_Attitude att - Time, Compass, Pitch, Roll, 
		LogFile.print(F("ATT"));
		LogTimeHeader();
		LogFile.print(F("HDG_T"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Pitch"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Roll"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Status"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("ROLL_Avg"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("HDG_Mag"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("HDG_Err"));
		LogFile.println();
	}

	if (Log_Situation & LoggingMask)
	{
		// Log_Situation sit - Time, BTW, DTW, CTE, CDA
		LogFile.print(F("SIT"));
		LogTimeHeader();
		LogFile.print(F("BTW"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DTW"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("CTE"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("CDA"));
}

	if (Log_Waypoints & LoggingMask)
	{
		// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon
		LogFile.print(F("WAY"));
		LogTimeHeader();
		LogFile.print(F("LAT0"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("LON0"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("LAT1"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("LON1"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("RLB"));
	}


	if (Log_Sailing & LoggingMask)
	{
		// Log_Sailing  sai - Time, CTS, HDG, BTW, WA,   CTE , Max CTE ,TackTime,TurnHDG, TargetHDG
		LogFile.print(F("SAI"));
		LogTimeHeader();
		LogFile.print(F("CTS"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("HDG_T"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("BTW"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("AWA"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("CTE"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("MaxCTE"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TT"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TurnHDG"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TargetHDG"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("CourseType"));
		LogFile.println();
	}

	if (Log_Navigation & LoggingMask)
	{
		// Log_Navigation nav - Time, BTW, DTW, RLB,
		LogFile.print(F("NAV"));
		LogTimeHeader();
		LogFile.print(F("BTW"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DTW"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("RLB"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("Sailable"));
	}

	if (Log_System & LoggingMask)
	{
		// Log_System = 0x0080; // sys, looptime
		LogFile.print(F("SYS"));
		LogTimeHeader();
		LogFile.print(F("Loop"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("FileSize"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Temp"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Press"));
		LogFile.println();
	}

	if (Log_GPS & LoggingMask)
	{
		// Log_GPS = 0x0100; // sys, GPS state information
		LogFile.print(F("GPS"));
		LogTimeHeader();
		LogFile.print(F("LAT"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("LON"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Last_Loc"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Last_Msg"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("SimulatedGPS"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("Loc_Valid"));
	}

	if (Log_Decisions & LoggingMask)
	{
		// Log_Decisions = 0x0200; //  
		LogFile.print(F("DEC"));
		LogTimeHeader();
		LogFile.print(F("MI"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("CS"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DE"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DER"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DEV"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("DEV2"));
		LogFile.println();
	}

	if (Log_ServoOut & LoggingMask)
	{
		// Log_ServoOut = 0x0400; // SVO values
		LogFile.print(F("SVO"));
		LogTimeHeader();
		LogFile.print(F("STR"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TTA"));
		LogFile.println();
	}

	if (Log_Voltages & LoggingMask)
	{
		// Log_Voltages = 0x0800; // VLT values
		LogFile.print(F("VLT"));
		LogTimeHeader();
		LogFile.print(F("PSV"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(F("PSI"));
	}

	if (Log_Usage & LoggingMask)
	{
		// Log_Usage  use - Time - 10 min counter, port rudder servo, stbd rudder, trim tab 
		//  = 0x1000;
		LogFile.print(F("USE"));
		LogTimeHeader();
		LogFile.print(F("10min"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("PortRudder"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("StarboardRudder"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TrimTabCounter"));
		LogFile.println();
	}

	if (Log_Environment & LoggingMask)
	{
		// Log_Usage  ENV - Time - TWD, AWA
		//  Log_Environment = 0x2000
		LogFile.print(F("ENV"));
		LogTimeHeader();
		LogFile.print(F("TWD"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("TWS"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("AWA"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Temp"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("Press"));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(F("WingAngle"));
		LogFile.println();
	}


	// Log the mission step  - Time, message
	LogFile.print(F("MIS"));
	LogTimeHeader();
	LogFile.print(F("mission_index"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("cmd"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("duration"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("boundary"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("controlMask"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("SteerAWA"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("TrimTabAngle"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("waypoint.lat"));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(F("waypoint.lng"));
	LogFile.println();

	// Log_change of parameter - Time, parameter index, 
	LogFile.print(F("PRS"));
	LogTimeHeader();
	LogFile.print("ParameterIndex");
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print("ParameterValue");
	LogFile.println();

	// log software version and other userful data at the start of each log file
	// log the OS version to the SD Card
	SD_Logging_Event_Messsage(Version);

	// log the usage values at start up.
	SD_Logging_Event_Usage(Configuration.SDCardLoggingMask);

	LogFile.flush();
}

void OpenSDLogFile(String LogFileName) {
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	LogFile.open(LogFileName.c_str(), O_CREAT | O_WRITE);
};

void LogTime(void)
{
	// V1.0 1/10/2016 John Semmens
	LogFile.print(Configuration.SDCardLogDelimiter); 
	LogFile.print(F("20"));
	LogFile.print(CurrentLocalTime.year);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CurrentLocalTime.month);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CurrentLocalTime.dayOfMonth);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CurrentLocalTime.hour);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CurrentLocalTime.minute);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(CurrentLocalTime.second);
	LogFile.print(Configuration.SDCardLogDelimiter);
}

void SD_Logging(word LoggingMask)
{
	// log data to the  SD Card data based on selected mask.
	// V1.0 4/8/2016 John Semmens
	// V1.1 3/9/2016 reorganised for logging to use a bit mask.
	// V1.2 1/11/2016 changed ATT to use True Hdg
	// V1.3 30/10/2017 added IMU status to Attitude
	// V1.4 10/12/2017 added Check_LogFileSize to this logging procedure, rather than calling separately.

	char FloatString[16];
	static BTStateType BTStatePrev;

	if (Log_Location & LoggingMask)
	{
		// Log_Location  loc - Time, Lat, Lon
		LogFile.print(F("LOC"));
		LogTime();
		LogFile.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5,FloatString));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.COG);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.SOG_mps); // m/s
		LogFile.println();
	}

	if (Log_Attitude & LoggingMask)
	{
		// Log_Attitude att - Time, Compass true, Pitch, Roll, 
		LogFile.print(F("ATT"));
		LogTime();
		LogFile.print(NavData.HDG);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.pitch);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.roll);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(myIMU.Algorithm_Status);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.ROLL_Avg);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.HDG_Mag);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.HDG_Err);
		LogFile.println();
	}

	if (Log_Situation & LoggingMask)
	{
		// Log_Situation sit - Time, BTW, DTW, CTE, CDA
		LogFile.print(F("SIT"));
		LogTime();
		LogFile.print(NavData.BTW);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.DTW);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.CTE);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.CDA);
		LogFile.println();
	}

	if (Log_Waypoints & LoggingMask)
	{
		// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon
		LogFile.print(F("WAY"));
		LogTime();
		LogFile.print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 10, 5, FloatString));

		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 10, 5, FloatString));

		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 10, 5, FloatString));

		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 10, 5, FloatString));

		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.RLB);
		LogFile.println();
	}

	if (Log_Sailing & LoggingMask)
	{
		// Log_Sailing  sai - Time, CTS, HDG, BTW, WA,   CTE , Max CTE ,TackTime,
		LogFile.print(F("SAI"));
		LogTime();
		LogFile.print(NavData.CTS);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.HDG);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.BTW);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.AWA);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.CTE);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.MaxCTE); 
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.TackDuration);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.TurnHDG);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.TargetHDG);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(CourseTypeToString(NavData.CourseType));
		LogFile.println();
	}

	if (Log_Navigation & LoggingMask)
	{
		// Log_Navigation nav - Time, BTW, DTW, BRL,
		LogFile.print(F("NAV"));
		LogTime();
		LogFile.print(NavData.BTW);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.DTW);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.RLB);
		LogFile.print(Configuration.SDCardLogDelimiter);

		if (NavData.IsBTWSailable) 
			LogFile.println("Y");
		else
			LogFile.println("N");
	}

	if (Log_System & LoggingMask)
	{
		// Log_System = 0x0080; // sys, looptime
		LogFile.print(F("SYS"));
		LogTime();
		LogFile.print(loop_period_us);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(LogFile.fileSize());
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.baro_temperature);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.baro_pressure);
		LogFile.println();
	}

	if (Log_GPS & LoggingMask)
	{
		// Log_GPS = 0x0100; // sys, GPS state information
		LogFile.print(F("GPS"));
		LogTime();
		LogFile.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(GPS_Last_Loc);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(GPS_Last_Message);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(UseSimulatedVessel);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(GPS_LocationIs_Valid(NavData.Currentloc));
	}

	if (Log_ServoOut & LoggingMask)
	{
		// Log_ServoOut = 0x0400; // SVO values
		LogFile.print(F("SVO"));
		LogTime();
		LogFile.print(SteeringServoOutput);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(WingSail.TrimTabAngle);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println();
	}

	if (Log_Voltages & LoggingMask)
	{
		// Log_Voltages = 0x0800; // VLT values
		LogFile.print(F("VLT"));
		LogTime();
		LogFile.print(PowerSensorV);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(PowerSensorI);
	}

	if (Log_Environment & LoggingMask)
	{
		// Log_Usage  ENV - Time - TWD, AWA
		//  Log_Environment = 0x2000
		LogFile.print(F("ENV"));
		LogTime();
		LogFile.print(NavData.TWD);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.TWS);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(NavData.AWA);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.baro_temperature);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print((int)myIMU.baro_pressure); 
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(WingSail.Angle);
		LogFile.println();
	}
	
	// look for a change in Bluetooth State and then log a message
	if (BTStatePrev != BTState)
	{
		LogFile.print(F("MSG"));
		LogTime();
		LogFile.print("BT");
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.println(GetBTStatus(BTState));
	}
	BTStatePrev = BTState;

	LogFile.flush();

	// Check if the current log file has reached or exceeded the configured file size limit
	Check_LogFileSize(CommandPort, Configuration.SDCardLoggingMask);

	// if GPS Time validity changes to valid then close and open the SD Card Logfile.
	// This is helpful because the timestamps on the previous file will be wrong.
	Check_GPS_TimeStatus(Configuration.SDCardLoggingMask);
}

void Check_LogFileSize(int CommandPort,word LoggingMask)
{
	// Check if the current log file has reached or exceeded the configured file size limit (in kb)
	// if the limit is reached then close and open a new file.
	// V1.0 11/10/2016 John Semmens

	if (LogFile.fileSize() >= (uint32_t(Configuration.MaxFileSize)*1024) ) {
		CloseThenOpenLogFile(LoggingMask);
	}
}

void CloseThenOpenLogFile(word LoggingMask)
{
	LogFile.close();
	SD_Logging_OpenFile(CommandPort, LoggingMask);
};


void SD_Logging_Event_Decisions(word LoggingMask)
{
	if (Log_Decisions & LoggingMask)
	{
		// Log_Decisions  dec - Time, command state, decision, decsion reason, and Decision event Value.
		LogFile.print(F("DEC"));
		LogTime();
		LogFile.print(StateValues.mission_index);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(CommandStateToString(StateValues.CommandState));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(DecisionEventToString(DecisionEvent));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(DecisionEventReasonToString(DecisionEventReason));
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(DecisionEventValue);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(DecisionEventValue2);
		LogFile.println();
	}
}

void SD_Logging_Event_ParameterChange(int ParameterIndex, char ParameterValue[12])
{
		// Log_change of parameter - Time, parameter index, 
		LogFile.print(F("PRS"));
		LogTime();
		LogFile.print(ParameterIndex);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(ParameterValue);
		LogFile.println();
}

void SD_Logging_Event_Messsage(String message)
{
	// Log a message  - Time, message
	LogFile.print(F("MSG"));
	LogTime();
	LogFile.print(message);
	LogFile.println();
}

void SD_Logging_Event_MissionStep(int mission_index)
{
	char FloatString[16];

	// Log the mission step  - Time, message
	LogFile.print(F("MIS"));
	LogTime();
	LogFile.print(mission_index);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(GetMissionCommandString(MissionValues.MissionList[mission_index].cmd));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].duration);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].boundary);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].controlMask);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].SteerAWA);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(MissionValues.MissionList[mission_index].TrimTabAngle);
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lat) / 10000000UL, 10, 5, FloatString));
	LogFile.print(Configuration.SDCardLogDelimiter);
	LogFile.print(dtostrf(float(MissionValues.MissionList[mission_index].waypoint.lng) / 10000000UL, 10, 5, FloatString));
	LogFile.println();
}


void SD_Logging_Event_Usage(word LoggingMask)
{
	if (Log_Usage & LoggingMask)
	{
		// Log_Usage  USE - Time, port rudder, starboard rudder, trim tab
		LogFile.print(F("USE"));
		LogTime();
		LogFile.print(VesselUsageCounters.intervalCounter);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(VesselUsageCounters.PortRudderCounter);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(VesselUsageCounters.StarboardRudderCounter);
		LogFile.print(Configuration.SDCardLogDelimiter);
		LogFile.print(VesselUsageCounters.TrimTabCounter);
		LogFile.println();
	}
}

void Check_GPS_TimeStatus(word LoggingMask)
{
	// monitor the GPS Time validity.
	// if it changes from invalid to valid then close and open the SD Card Logfile.
	// This is helpful because the timestamps on the previous file will be wrong.
	// V1.0 29/4/2019 John Semmens

	static bool PrevStatus;
	bool Status = GPS_TimeIs_Valid();

	if (Status && !PrevStatus)
	{
		CloseThenOpenLogFile(LoggingMask);
	}

	PrevStatus = Status;
}