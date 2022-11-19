// OLED Display handler
// Display data pages for Voyager OS.
// V1.1 24/1/2018 added Page L for Loiter Data
// V1.2 10/11/2018 added Page b for bluetooth
// V1.3 11/4/2020 added i page for Nav #2 data
// V1.4 1/2/2021 added Running Laylines to Q Sailing Data.


#include "OLED_Logging.h"


/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/category/63_98

This example is for a 128x32 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#include "Waypoints.h"
#include "Wire.h"

#include "Navigation.h"
#include "RadioControl.h"
#include "CommandState_Processor.h"
#include "GPS.h"
#include "USFS_IMU.h"
#include "configValues.h"
#include "Loiter.h"
#include "BluetoothConnection.h"
#include "MPU9250.h"
#include "WindAngle_2950.h"
#include "Wingsail.h"
#include "WearTracking.h"
#include "location.h"
#include "TelemetryLogging.h"
#include "sim_vessel.h"
#include "sim_weather.h"

extern NavigationDataType NavData;
extern StateValuesStruct StateValues;
extern IMUStruct myIMU;
extern char Version[];
extern char VersionDate[];
extern Time CurrentLocalTime;
extern RC_IN_Type RC_IN;
extern HardwareSerial *Serials[];
extern bool SD_Card_Present; // Flag for SD Card Presence
extern configValuesType Configuration;
extern int CommandPort;
extern uint32_t CommandPortBaudRate;
extern MissionValuesStruct MissionValues;

extern char MessageDisplayLine1[10];
extern char MessageDisplayLine2[10];

extern LoiterStruct LoiterData;

extern double PowerSensorV, PowerSensorI;

extern byte BluetoothStatePin;
extern BTStateType BTState;

extern MPU9250 WingSailAngleSensor;		// The IMU including Magnetic Compass and Gyro
extern WingSailType WingSail;

extern VesselUsageCountersStruct VesselUsageCounters;

extern byte OLED_PowerPin;
extern sim_vessel simulated_vessel;
extern double SteeringServoOutput;
extern sim_weather simulated_weather;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif



static const byte OLED_Address = 0x3C;
bool OLED_Present = false;


void OLED_Init(int SerialPortNumber) {

	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	display.begin(SSD1306_SWITCHCAPVCC, OLED_Address);  // initialize with the I2C addr 0x3C (for the 128x32)
												// init done

												// Show image buffer on the display hardware.
												// Since the buffer is intialized with an Adafruit splashscreen
												// internally, this will display the splashscreen.
	display.display();

	// Clear the buffer.
	display.clearDisplay();



	byte error;
	Wire.beginTransmission(OLED_Address);
	error = Wire.endTransmission();

	if (error == 0)
	{
		(*Serials[SerialPortNumber]).print(F("MSG,OLED found at address: 0x"));
		(*Serials[SerialPortNumber]).println(OLED_Address, HEX);
		OLED_Present = true;

		display.setTextSize(1);
		display.setTextColor(WHITE);
	}
	else
	{
		(*Serials[SerialPortNumber]).print(F("MSG,No OLED found at address: 0x"));
		Serial.println(OLED_Address, HEX);
		OLED_Present = false;
	}
};

void OLED_Logging(char LoggingLevel)
{
	// Update the OLED screen with data in accordance with the current logging level.
	// V1.0 10/12/2017 John Semmens
	// V1.1 20/1/2019 added Mag Accuracy to the Attitude page.
	// V1.2 18/2/2019 added Wear Statistics

	// values for LCD Logging:
	// 0: Off

	// a: Attitude
	// b: Bluetooth   
	// c: Compass details
	// d: display message from the voyager base station
	// e: Equipment
	// f: /// Check List // not implemented
	// g: GPS Data
	// h: Home Location
	// i: Navigation #2
	// j: Wear Statistics
	// k: Wind Situation
	// l: Loiter
	// m: Mission
	// n: Navigation
	// o: Current Mission Step
	// p: GPS Detail Data 
	// q: Sail Navigation Parameters #2
	// r: RC Input
	// s: Situation
	// t: Timing display including millis()
	// U: Sail Navigation Parameters #1
	// V: Version/Time Display 
	// w: Waypoints
	// x: Wing sail 
	// y: Wing Sail Angle Sensor
	// z: System Voltages

	// 1: Sailable
	// 2: TWD Calculation
	// 3: Favoured Tack Calculation
	// 4: Simulator Data


	// assume the Display may have disabled, so enable on each command
	display.ssd1306_command(SSD1306_DISPLAYON);

	// Blank Display **********************************
	if (LoggingLevel == '0' || !OLED_Present)
	{
		// Clear the buffer.
		display.clearDisplay();
		display.display();
		display.ssd1306_command(SSD1306_DISPLAYOFF);
	}

	// Version Display **********************************
	if (LoggingLevel == 'v' && OLED_Present) // v = Version/Time
	{
		// display Version Information for Startup Screen
		display.clearDisplay();
		display.setTextSize(2);
		display.setCursor(0, 0);

		display.println(Version);

		display.print(F("Released: "));
		display.println(VersionDate);

		display.display();
	}

	// Equipment Display **********************************
	if (LoggingLevel == 'e' && OLED_Present) // e = equipment
	{
		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0, 0);

		// Row 1 
		display.print(F("SD Card:"));
		if (SD_Card_Present)
			display.println(F(" OK"));
		else
			display.println(" -");

		// Row 2 -- WingSailAngleSensor
		display.print(F("Wingsail Sensor:"));
		if (WingSailAngleSensor.Initialsed)
			display.print(F(" OK"));
		else
			display.print(F(" Fail"));
		display.println();

		// Row 3 -- Command channel
		display.print(F("Cmd Ch: "));
		display.print(CommandPort);
		display.print(F(": "));
		display.print(CommandPortBaudRate);
		display.print(F(" Bd"));
		display.println();

		display.display();
	}

	// RC Input Display **********************************
	if (LoggingLevel == 'r' && OLED_Present) // r = RC Input
	{
		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0, 0);

		// Row 1 -- RC In Channel0
		display.print(F("RC0:"));
		display.print(RC_IN.Channel[0]);
		display.print(" ");
		display.print(F("Cmd Sw:"));
		display.println(RC_IN.RC_Command_Switch_Position);

		// Row 2 -- RC In Channel1
		display.print(F("RC1:"));
		display.println(RC_IN.Channel[1]);

		// Row 3 -- RC In Channel2
		display.print(F("RC2:"));
		display.println(RC_IN.Channel[2]);

	//	// Row 4 -- RC In Channel3
		display.print(F("RC3:"));
		display.print(RC_IN.Channel[3]);
		display.print("  ");
		display.print(F("Valid: "));
		display.println(RC_IN.SignalValid);

		display.display();
	}

	// Attitude Display **********************************
	if (LoggingLevel == 'a' && OLED_Present) // a = attitude
	{
		// display compass data  and calibration limits

		// line 1 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.print("M");

		display.print((int)wrap_360(myIMU.heading + Configuration.CompassOffsetAngle));
	//	display.print(char(0x09));
		display.print(" ");
		display.print("T");
		display.println((int)NavData.HDG);

	//	display.print(char(0x09));
		display.setTextSize(1);
		// line 3
		display.print("P");
		display.print((int)myIMU.pitch);
	//	display.print(char(0x09));
		display.print("  ");
		// line 3
		display.print("R");
		display.print((int)myIMU.roll);
	//	display.print(char(0x09));
		display.print(F("  Stat:"));
		display.println(myIMU.Algorithm_Status); // 8 is good

		display.setTextSize(1);
		// line 4

		display.print("  ");
		display.print((int)myIMU.baro_temperature);
		display.print(char(0x09));
		display.print("C  ");
		display.print((int)myIMU.baro_pressure); 
		display.print("mbar ");
		display.display();
	}

	// compass detail Display **********************************
	if (LoggingLevel == 'c' && OLED_Present) // c = compass details
	{
		char MsgString[16];

		// Row 1 -- current lat/lon
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		display.print("Mag Hdg:");
		display.print(NavData.HDG_Mag);

		display.print(" WA:");
		display.print(NavData.AWA);
		display.println();

		display.print("Hdg Err: ");
		display.println(NavData.HDG_Err);

		display.print("Hdg    : ");
		display.println(NavData.HDG);

		display.print("COG: ");
		display.print(NavData.COG);
		display.print(" SOG: ");
		display.print(NavData.SOG_mps);
		display.println();

		display.display();
	}

	// GPS Display **********************************
	if (LoggingLevel == 'g' && OLED_Present) // g = GPS
	{
		char MsgString[16];

		// Row 1 -- current lat/lon
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.print("P");
		display.println(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));
		display.print(" ");
		display.println(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));

		//// Row 2 -- COG/SOG
		//display.print("COG:");
		//display.print(NavData.COG);
		//display.print("  ");

		//display.print("SOG:");
		//display.println(NavData.SOG);

		display.display();
	}

	// GPS Detail Display **********************************
	if (LoggingLevel == 'p' && OLED_Present) // p = GPS Detail
	{
		char MsgString[16];

		// Row 1 -- current lat/lon
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		//display.print("P");
		display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));
		display.print(" ");
		display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));
		display.println();

		// Row 2 -- COG/SOG
		display.print("COG:");
		display.print(NavData.COG);
		display.print("  ");

		display.print("SOG:");
		display.print(NavData.SOG_mps);
		display.println(" m/s ");

		// Row 3 -- Time
		display.print(CurrentLocalTime.hour);
		display.print(":");
		display.print(CurrentLocalTime.minute);
		display.print(":");
		display.print(CurrentLocalTime.second);

		display.print(" ");

		// Row 4 -- date
		display.print(CurrentLocalTime.dayOfMonth);
		display.print("/");
		display.print(CurrentLocalTime.month);
		display.print("/");
		display.print(CurrentLocalTime.year + 2000);

		display.display();
	}


	// Navigation Display **********************************
	if (LoggingLevel == 'n' && OLED_Present) // n = Navigation
	{
		char MsgString[16];

		// Row 1 -- current lat/lon
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));

		display.print(" ");
		display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));
		display.println();

		// Row 2 -- prev WP
		display.print(F("THG:"));
		display.print(NavData.TargetHDG);
		display.print("  ");

		display.print(F("BTW:"));
		display.print(NavData.BTW);
		display.print(" ");

		display.print("M:");
		display.println(StateValues.mission_index);

		// Row 3 -- next WP
		display.print(F("HDG: "));
		display.print(NavData.HDG);
		display.print("  ");

		display.print(F("Past WP:"));
		if (NavData.PastWP)
		{
			display.print(F("YES"));
		}
		else
		{
			display.print(F("NO "));
		}
		display.println();

		// Row 4 -- DTW,CTE
		display.print(F("DTW: "));
		display.print(NavData.DTW);
		display.print("  ");

		display.print(F("CTE: "));
		display.println(NavData.CTE);

		display.display();
	}

	// Navigation Display #2 **********************************
	if (LoggingLevel == 'i' && OLED_Present) // i = Navigation #2
	{
		char MsgString[16];

		// Row 1 -- 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		
		display.print(F("HDG: "));
		display.print(NavData.HDG);
		display.print(" ");
		display.print(F("THG:"));
		display.print(NavData.TargetHDG);
		display.println();

		// Row 2 -- 
 
		display.print(F("DTW: "));
		display.print(NavData.DTW);
		display.print(" ");
		display.print(F("MaxCTE:"));
		display.print(NavData.MaxCTE);
		display.println();

		// Row 3 --
		display.print(F("BTW: "));
		display.print(NavData.BTW);
		display.print(" ");
		display.print(F("CTE: "));
		display.print(NavData.CTE);
		display.println();

		// Row 4 -- DTW,CTE
		display.print(F("CTS: "));
		display.print(NavData.CTS);
		display.print(" ");
		display.print(F("Adj: "));
		display.print(NavData.CTE_Correction);
		display.println();

		display.display();
	}


	// Current Mission Step Display **********************************
	if (LoggingLevel == 'o' && OLED_Present) //o = Current Mission Step
	{
		char MsgString[16];

		// Row 1 -- mission index and duration
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		display.print("M:");
		display.print(StateValues.mission_index);

		display.print(" Cmd:"); // 
		display.print(GetMissionCommandString(MissionValues.MissionList[StateValues.mission_index].cmd));
		display.println();

		// Row 2 -- line 2
		display.print("Dur:"); // in minutes
		display.print(MissionValues.MissionList[StateValues.mission_index].duration);

		display.print(" MT:  "); // in seconds
		display.print((millis() - MissionValues.MissionCommandStartTime) / 1000);
		display.println();

		// Row 3 -- line 3
		display.print("AWA: ");
		display.print(MissionValues.MissionList[StateValues.mission_index].SteerAWA);
		display.print(" TTA: ");
		display.print(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
		display.println();

		// Row 4 -- 
		display.print("C:");
		display.print(NavData.CTS);
		display.print(" H:");
		display.print(NavData.HDG);
		display.print(" A:");
		display.print(NavData.AWA);
		display.println();

		display.display();
	}

	// Situation Display **********************************
	if (LoggingLevel == 's' && OLED_Present) // s = Situation
	{ //BTW, DTW,-- CTS, CTE, -- CDA, MI ,-- WA (wind angle), HDG True

		// Row 1 -- BTW, DTW
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		display.print(F("BTW:"));
		display.print(NavData.BTW);
		display.print(" ");

		display.print(F("DTW:"));
		display.println(NavData.DTW);

		// Row 2 -- CTS, CTE
		display.print(F("CTS:"));
		display.print(NavData.CTS);
		display.print(" ");

		display.print(F("CTE: "));
		display.println(NavData.CTE);

		// Row 3 -- CDA MI
		display.print(F("CDA: "));
		display.print(NavData.CDA);
		display.print("  ");

		display.print(F("MI:"));
		display.println(StateValues.mission_index);

		// Row 4 --  HDG True, WA (wind Angle)
		display.print(F("HDG: "));
		display.print((int)NavData.HDG);
		display.print("  ");

		display.print(F("WA: "));
		display.println(NavData.AWA);

		display.display();
	}

	// Mission Display **********************************
	if (LoggingLevel == 'm' && OLED_Present) // m = Command
	{
		// Row 1 -- BTW, DTW
		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0, 0);

		display.print(F("BTW:"));
		display.print(NavData.BTW);
		display.print(" ");
		display.print(F("DTW:"));
		display.println(NavData.DTW);

		display.setTextSize(1);

		// Row 2--  HDG True
		display.print(F("HDG:"));
		display.print((int)NavData.HDG);
		display.print(" ");

		display.print(F("LOC: "));
		if (GPS_LocationIs_Valid(NavData.Currentloc)) {
			display.print("OK");
		}
		else {
			display.print("NO");
		}
		display.println();

		// Row 3--  Mission Index
		display.print(F("MI:"));
		display.print(StateValues.mission_index);
		display.print("/");
		display.print(MissionValues.mission_size);
		display.print(" ");
		display.print(F("MaxCTE:"));
		display.print(NavData.MaxCTE);
		display.println();

		// Row 4 -- Command State
		display.print(StateValues.CommandState);
		display.print(" ");
		switch (StateValues.CommandState)
		{
		case  vcsIdle:
			display.print(F("Idle          "));
			break;

		case  vcsFullManual:
			display.print(F("Full Manual   "));
			break;

		case  vcsPartialManual:
			display.print(F("Part Manual   "));
			break;

		case  vcsResetMissionIndex:
			display.print(F("ResetMissionIndex"));
			break;

		case  vcsSetHome:
			display.print(F("Set Home       "));
			break;

		case  vcsSteerMagneticCourse:
			display.print(F("Steer Compass "));
			display.print(StateValues.SteerCompassBearing);
			break;

		case  vcsSteerWindCourse:
			display.print(F("Steer Wind "));
			display.print(StateValues.SteerWindAngle);
			display.print(" ");
			display.print(Configuration.TrimTabDefaultAngle);
			break;

		case  vcsFollowMission:
			display.print(F("Follow Mission"));
			break;

		case  vcsReturnToHome:
			display.print(F("Return To Home"));
			break;

		case vcsLoiter:
			display.print(F("Loiter Here    "));
			break;

		default:
			display.print(F("Unknown"));
		}
		display.display();
	}

	// Waypoint Display **********************************
	if (LoggingLevel == 'w' && OLED_Present) // w = Waypoints
	{
		char MsgString[16];

		// Row 1 -- current lat/lon
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		display.print("L");
		display.print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 9, 5, MsgString));

		display.print("  ");
		display.print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 9, 5, MsgString));

		// Row 2 -- prev_WP lat/lon

		display.print("P");
		display.print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 9, 5, MsgString));

		display.print("  ");
		display.print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 9, 5, MsgString));

		// Row 3 -- next_WP
		display.print("N");
		display.print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 9, 5, MsgString));

		display.print("  ");
		display.print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 9, 5, MsgString));

		// Row 4
		display.print("M:");
		display.print(StateValues.mission_index);
		display.print("   ");

		display.print(F("Past WP:"));
		if (NavData.PastWP)
		{
			display.print("YES");
		}
		else
		{
			display.print("NO ");
		}
		display.display();
	}


	// Home Location Display **********************************
	if (LoggingLevel == 'h' && OLED_Present) // h = Home Location 
	{
		char MsgString[16];
		// Row 1 -- Home 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.print("H");

		// Row 2 -- Home lat/lon
		display.println(dtostrf(float(StateValues.home.lat) / 10000000UL, 9, 5, MsgString));

		display.setTextSize(1);
		if (StateValues.home_is_set)
		{
			display.print("Yes");
		}
		else
		{
			display.print("NO");
		}

		display.setTextSize(2);
		display.println(dtostrf(float(StateValues.home.lng) / 10000000UL, 9, 5, MsgString));

		display.display();
	}


	// Display Message  **********************************
	if (LoggingLevel == 'd' && OLED_Present) // d = display
	{
		// Row 1 -- line 1 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);

		display.println(MessageDisplayLine1);

		// Row 2 -- line 2
		display.println(MessageDisplayLine2);

		display.display();
	}

	// Display Message  **********************************
	if (LoggingLevel == 'l' && OLED_Present) // l = Loiter
	{
		char MsgString[16];

		// Row 1 -- Loiter State
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		display.print(F("Loiter: "));
		switch (LoiterData.LoiterState)
		{
		case lsNotLoitering:
			display.println(F("NotLoitering"));
			break;

		case lsApproach:
			display.println(F("Approach"));
			break;

		case lsPortSide:
			display.println(F("PortSide"));
			break;

		case lsStarboardSide:
			display.println(F("StbdSide"));
			break;
		default:;
		}

		// Row 2 -- Loiter Point
		display.print("L");
		display.print(dtostrf(float(LoiterData.loiterCentreLocation.lat) / 10000000UL, 9, 5, MsgString));

		display.print("  ");
		display.print(dtostrf(float(LoiterData.loiterCentreLocation.lng) / 10000000UL, 9, 5, MsgString));

		// Row 3 -- Bearing and Distance
		display.print("BTW:");
		display.print(LoiterData.BTW);
		display.print(" Dist:");
		display.println(LoiterData.DTW);

		// Row 4 -- Manoeuvre and Turn Heading 
		display.print("Move:");
		switch (NavData.Manoeuvre)
		{
		case mtNotDefined:
			display.print("N/A");
			break;
		case mtTack:
			display.print("Tack");
			break;
		case mtGybe:
			display.print("Gybe");
			break;
		default:;
		}
		display.print(" TurnHDG:");
		display.println(NavData.TurnHDG);

		display.display();
	}

	// Display Timing information  **********************************
	if (LoggingLevel == 't' && OLED_Present) // t = timing
	{
		// Row 1 -- line 1 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);
		
		//unsigned long totalms = millis();

		display.print("millis(): ");  // in seconds
		display.println(millis()/1000);

		// Row 2 -- line 2
		display.print("MC Start: "); // in seconds
		display.println(MissionValues.MissionCommandStartTime/1000);

		// Row 3 -- line 3
		display.print("Elapsed:  "); // in seconds
		display.println((millis() - MissionValues.MissionCommandStartTime)/1000);

		// Row 4 -- 
		display.print("Duration m: "); // in minutes
		display.println( MissionValues.MissionList[StateValues.mission_index].duration   );


		display.display();
	}

	// Display System Voltages   **********************************
	if (LoggingLevel == 'j' && OLED_Present) // j = Wear Statistics
	{
		char MsgString[16];


		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 
		display.print(F("minutes: "));
		display.print(VesselUsageCounters.intervalCounter * 10);
		display.println();

		// Row 2 
		display.print(F("Port Rudder:"));
		display.print(VesselUsageCounters.PortRudderCounter);
		display.println();

		// Row 3 
		display.print(F("Stbd Rudder:"));
		display.print(VesselUsageCounters.StarboardRudderCounter);
		display.println();

		// Row 4 
		display.print(F("Trim Tab:"));
		display.print(VesselUsageCounters.TrimTabCounter);
		display.println();

		display.display();
	}


	// Display System Voltages   **********************************
	if (LoggingLevel == 'z' && OLED_Present) // z = Voltages
	{
		char MsgString[16];
		
		// Row 1 -- line 1 
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		display.print("Pwr V:");
		display.print(dtostrf(PowerSensorV, 8, 3, MsgString));
		display.println(" Vdc");

		// Row 1 -- line 1
		display.print("Pwr I:");
		display.print(dtostrf(PowerSensorI, 8, 2, MsgString));
		display.println(" mA");

		// Row 3 -- line 3
		// Row 4 -- 
		display.display();
	}

	// Bluetooth Status Display **********************************
	if (LoggingLevel == 'b' && OLED_Present) // b = Bluetooth
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		//	char MsgString[16];

		// Row 1 -- Bluetooth Status
		display.print(F("BT Port "));
		display.print(Configuration.BluetoothPort);
		display.print(" :  ");
		display.println(Configuration.BTPortBaudRate);

		// Row 2 -- Bluetooth
		display.print(F("Status Pin: "));
		display.print(BluetoothStatePin);
		display.print(": ");
		display.println(digitalRead(BluetoothStatePin));

		// Row 3 -- Bluetooth status
		display.print(F("State: "));
		display.print(BTState);
		display.print("  ");
		display.println(GetBTStatus(BTState));

		// Row 4 -- Bluetooth
		display.print("--: ");
		display.println(" ");

		display.display();
	}

	// Wingsail Display **********************************
	if (LoggingLevel == 'x' && OLED_Present) // x= WingSail 
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- Wingsail Angle
		display.print(F("WS Angle:"));
		display.print(WingSail.Angle); //WingSailAngleSensor.MagneticAngle

		// show which tack we're on
		display.print(" ");
		switch (WingSail.Tack)
		{
		case wsPortTack:
			display.print(F("Port Tk"));
			break;
		case wsStarboardTack:
			display.print(F("Stbd Tk"));
			break;
		case wsHeadToWind:
			display.print(F("Hd to Wnd"));
			break;
		default:;
		}
		display.println();

		//// Row 2 -- Wingsail state (fwd/idle/rev)
		display.print(F("State: "));
		switch (WingSail.State)
		{
		case wsForward:
			display.print(F("Forward"));
			break;
		case wsIdle:
			display.print(F("Idle"));
			break;
		case wsReverse:
			display.print(F("Reverse"));
			break;
		default:;
		}
		display.println();

		//// Row 3 -- Trim Tab Angle
		display.print(F("Tab Angle: "));
		display.print(WingSail.TrimTabAngle);
		display.println();

		//// Row 4 -- Servo us
		display.print(F("Servo us: "));
		display.print(WingSail.Servo_microseconds);
		display.println(" ");

		display.display();
	}

	// Wingsail Angle Sensor Display **********************************
	if (LoggingLevel == 'y' && OLED_Present) // y= WingSail AngleSensor
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- Wingsail Angle
		display.print(F("WS Angle:"));
		display.println(WingSailAngleSensor.MagneticAngle);

		// Row 2 -- mx / my
		display.print("mx: ");
		display.print((int)WingSailAngleSensor.mx);
		display.print(" my: ");
		display.println((int)WingSailAngleSensor.my);

		//// Row 3 --  min / max X
		display.print("X Min:");
		display.print((int)WingSailAngleSensor.mx_min);
		display.print("Max:");
		display.print((int)WingSailAngleSensor.mx_max);
		display.println();

		//// Row 4 -- min / max Y
		display.print("Y Min:");
		display.print((int)WingSailAngleSensor.my_min);
		display.print("Max:");
		display.print((int)WingSailAngleSensor.my_max);
		display.println();

		display.display();
	}

	// Wind Situation Display **********************************
	if (LoggingLevel == 'k' && OLED_Present) // k=Wind Situation
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- Wingsail Angle
		display.print(F("WS Ang:"));
		display.print(WingSailAngleSensor.MagneticAngle);

		display.print(F(" AWA:"));
		display.print(NavData.AWA);
		display.println();

		// Row 2 HDG
		display.print(F("HDG:"));
		display.print(NavData.HDG);

		display.print(F(" "));
		display.print(CourseTypeToString(NavData.ManoeuvreState));

		display.println();

		// Row 3 -- TWD TWS
		display.print(F("TWD: "));
		display.print((int)NavData.TWD);
		display.print(F(" TWS: "));
		display.print((int)NavData.TWS);
		display.print(F("m/s"));
		display.println();

		// Row 4 --  CTS
		display.print(F("TurnHDG:"));
		display.print(NavData.TurnHDG);
		display.print(F(" CTS:"));
		display.print(NavData.CTS);
		display.println();

		display.display();
	}

	// TWD True Wind Direction Calculation Display **********************************
	if (LoggingLevel == '2' && OLED_Present) // 2 = TWD Calculation
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- Wingsail Angle, AWA
		display.print(F("WS Ang:"));
		display.print(WingSailAngleSensor.MagneticAngle);

		display.print(F(" AWA:"));
		display.print(NavData.AWA);
		display.println();

		// Row 2 HDG AWD
		display.print(F("HDG:"));
		display.print(NavData.HDG);

		display.print(F(" AWD:"));
		display.print(NavData.AWD);
		display.println();

		// Row 3 -- TWD 
		display.print(F("TWD: "));
		display.print((int)NavData.TWD);

		display.println();

		// Row 4 --  TWD Offset
		display.print(F("TWD Offset:"));
		display.print(NavData.TWD_Offset);

		display.println();

		display.display();
	}


	//Favoured Tack Calculation  **********************************
	if (LoggingLevel == '3' && OLED_Present) // 3=Favoured Tack Calculation
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- HDG, BTW
		display.print(F("HDG:"));
		display.print(NavData.HDG);
		display.print(F(" BTW:"));
		display.print(NavData.BTW);
		display.println();

		// Row 2 TWD
		display.print(F("TWD: "));
		display.print((int)NavData.TWD);
		display.println();

		// Row 3 Favoured Tack
		display.print(F("Fav Tack: "));
		switch (NavData.FavouredTack)
		{
		case SteeringCourseType::ctDirectToWayPoint:
			display.print(F("Direct"));
			break;

		case SteeringCourseType::ctPortTack:
			display.print(F("Port"));
			break;

		case SteeringCourseType::ctStarboardTack:
			display.print(F("Stdb"));
			break;

		case SteeringCourseType::ctPortTackRunning:
			display.print(F("PortRun"));
			break;

		case SteeringCourseType::ctStarboardTackRunning:
			display.print(F("StdbRun"));
			break;

		default:;
		}

		display.println();

		// Row 4 --  Course Type
		display.print(F("Course Type: "));
		switch (NavData.CourseType)
		{
		case SteeringCourseType::ctDirectToWayPoint:
			display.print(F("Direct"));
			break;

		case SteeringCourseType::ctPortTack:
			display.print(F("Port"));
			break;

		case SteeringCourseType::ctStarboardTack:
			display.print(F("Stdb"));
			break;

		case SteeringCourseType::ctPortTackRunning:
			display.print(F("PortRun"));
			break;

		case SteeringCourseType::ctStarboardTackRunning:
			display.print(F("StdbRun"));
			break;

		default:;
		}
		display.println();

		display.display();
	}


	// IsSailable Display **********************************
	if (LoggingLevel == '1' && OLED_Present) // 1= IsSailable
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- Wingsail Angle
		display.print(F("AWATW: "));
		display.print((int)NavData.WindAngleToWaypoint);

		display.print(F(" AWA:"));
		display.print(NavData.AWA);
		display.println();

		// Row 2 HDG
		display.print(F("HDG:"));
		display.print(NavData.HDG);

		display.print(F(" Margin: "));
		display.print(Configuration.SailableAngleMargin);

		display.println();

		// Row 3 -- Config
		display.print(F("MinUp: "));
		display.print(Configuration.MinimumAngleUpWind);
		display.print(F(" MinDn: "));
		display.print(Configuration.MinimumAngleDownWind);
		display.println();

		// Row 4 --  
		display.print(F("BTW:"));
		display.print(NavData.BTW);

		display.print(F(" Sailable:"));
		if (NavData.IsBTWSailable)
			display.print("Y");
		else
			display.print("N");

		display.println();

		display.display();
	}


	// 	Sail Navigation Parameters **********************************
	if (LoggingLevel == 'U' && OLED_Present) // 	// u: Sail Navigation Parameters
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1); 

		// Row 1 -- BTW AWA
		display.print(F("BTW:"));
		display.print(NavData.BTW);

		display.print(F(" Sailable:"));
		if (NavData.IsBTWSailable)
			display.print("Y");
		else
			display.print("N");
		display.println();

		// Row 2 -- 
		display.print(F("TWD:"));
		display.print(NavData.TWD);


		display.print(F(" CTS:"));
		display.print(NavData.CTS);

		display.println();

		// Row 3 --
		display.print(F("AWA:"));
		display.print(NavData.AWA);

		display.print(" ");
		switch (NavData.PointOfSail)
		{
		case PointOfSailType::psNotEstablished:
			display.print(F("N/A"));
			break;

		case PointOfSailType::psPortTackBeating:
			display.print(F("Port Beat"));
			break;

		case PointOfSailType::psPortTackRunning:
			display.print(F("Port Run"));
			break;

		case PointOfSailType::psStarboardTackBeating:
			display.print(F("Stbd Beat"));
			break;

		case PointOfSailType::psStarboardTackRunning:
			display.print(F("Stbd Run"));
			break;

		default:;
		}

		display.println();
		

		// Row 4 --  
		display.print(F("HDG:"));
		display.print(NavData.HDG);
		display.print(F(" AWATW: "));
		display.print((int)NavData.WindAngleToWaypoint);
		display.println();

		display.display();
	}

	// 	Sail Navigation Parameters #2 **********************************
	if (LoggingLevel == 'q' && OLED_Present) // 	// q: Sail Navigation Parameters
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(1);

		// Row 1 -- BTW AWA
		display.print(F("TWD:"));
		display.print(NavData.TWD);

		display.print(F(" CTE:"));
		display.print(NavData.CTE);
		display.println();

		// Row 2 -- 
		display.print(F("PL:"));
		display.print(NavData.PortLayline);
		display.print(" ");
		display.print(NavData.PortLaylineRunning);



		display.print(F(" SL:"));
		display.print(NavData.StarboardLayline);
		display.print(" ");
		display.print(NavData.StarboardLaylineRunning);
		display.println();

		// Row 3 --
		display.print(F("CTS:"));
		display.print(NavData.CTS);

		display.print(F(" Tack:"));
		switch (NavData.CourseType)
		{
		case SteeringCourseType::ctDirectToWayPoint:
			display.print(F("Direct"));
			break;

		case SteeringCourseType::ctPortTack:
			display.print(F("Port"));
			break;

		case SteeringCourseType::ctStarboardTack:
			display.print(F("Stdb"));
			break;

		case SteeringCourseType::ctPortTackRunning:
			display.print(F("PortRun"));
			break;

		case SteeringCourseType::ctStarboardTackRunning:
			display.print(F("StdbRun"));
			break;

		default:;
		}
		display.println();

		// Row 4 --  
		display.print(F("BTW:"));
		display.print(NavData.BTW);

		display.print(F(" Sailable:"));
		if (NavData.IsBTWSailable)
			display.print("Y");
		else
			display.print("N");

		display.println();

		display.display();
	}



// Simulation  Data Display **********************************
if (LoggingLevel == '4' && OLED_Present) // 1= IsSailable
{
	display.clearDisplay();
	display.setCursor(0, 0);
	display.setTextSize(1);

	// Row 1 -- time
	display.print(F("ms: "));
	display.print(simulated_vessel.update_time_ms);

	display.println();

	// Row 2 HDG
	display.print(F("HDG:"));
	display.print(NavData.HDG);

	display.print(F(" Wind:"));
	display.print(simulated_weather.WindDirection);

	display.println();

	// Row 3 -- steeering
	display.print(F("SVO:"));
	display.print((int)SteeringServoOutput);
	display.print(F(" Ctr:"));
	display.print((int)Configuration.pidCentre);
	display.println();

	// Row 4 --  
	display.print(F("SOG: "));
	display.print(simulated_vessel.SOG_mps);

	display.print(F("WS Angle:"));
	display.print(WingSail.Angle);

	display.println();

	display.display();
}

};






String GetMissionCommandString(MissionCommandType cmd)
{
	// function to return a string version of the MissionCommandType enumnerated type
	// V1.0 29/5/2019 John Semmens
	String MissionCommandString;

	switch (cmd)
	{
	case MissionCommandType::ctGotoWaypoint:
		MissionCommandString = F("Waypoint");
		break;

	case MissionCommandType::ctLoiter:
		MissionCommandString = F("Loiter");
		break;

	case MissionCommandType::ctLoiterUntil:
		MissionCommandString = F("Loiter'til");
		break;

	case MissionCommandType::ctReturnToHome:
		MissionCommandString = F("Home");
		break;

	case MissionCommandType::ctSteerWindCourse:
		MissionCommandString =  F("SteerWind");
		break;

	default:
		MissionCommandString = F("unknown");
	}

	return MissionCommandString;
}