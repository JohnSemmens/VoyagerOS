// Command Processor 
// Interpret commands received through the main serial port
// V1.0 22/12/2015
// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 8/10/2016 updated to add mission command: ctLoiterUntil
// V1.3 18/10/2016 added command scg - Command State Get.
// V1.4 23/10/2016 expanded config parameter list up to 44.
// V1.5 12/11/2016 corrected logging masks to be long rather than int.
// v1.6 29/10/2017 updated for change from MPU-2950 to BNO-055
// V1.7 12/11/2017 added hlc command to clear home location
//				   added mcp command for retrieving the mission from the autopilot for plotting
// V1.8 24/1/2018 added parameter LoiterRadius for use in the "Loiter Here" command
// V1.9 2/4/2018 added Parameters for Voltage Measurement scale factors.
//					added vmg, Get Voltage Measurements
// V1.10 24/4/2018 added parameter pidCentre and bug fix for Target Heading filter constant - float not int
// V1.11 5/5/2019 added support for parameter for the default Trim Tab angle. 
// V1.12 29/6/2019 added CCS and CCG for Setting Campass Cal, and Getting Compass Status
// V1.13 16/7/2019 adding the new Mission command for Steering a Wind Angle

#include "CommandState_Processor.h"
#include "Mission.h"
#include "CLI.h"
#include "configValues.h"
#include "TinyGPS++.h"
#include "GPS.h"
#include "USFS_IMU.h"
#include "LEDHeartBeat.h"
#include "Navigation.h"
#include "Steering.h"
#include "WindAngle_2950.h"
#include "MPU9250.h"
#include "BluetoothConnection.h"
#include "PowerManagement.h"
#include "TelemetryLogging.h"
#include "SDCardLogFile.h"
#include "Wingsail.h"
#include "TelemetryMessages.h"
#include "location.h"
#include "sim_vessel.h"

extern Time CurrentLocalTime;
extern NavigationDataType NavData;
extern TinyGPSPlus gps;
extern char Version[];
extern char VersionDate[];
extern configValuesType Configuration;
extern MissionValuesStruct MissionValues;
extern StateValuesStruct StateValues;
extern bool UseSimulatedVessel;
extern IMUStruct myIMU;
extern HardwareSerial *Serials[];

extern char MessageDisplayLine1[10];
extern char MessageDisplayLine2[10];

extern double PowerSensorV, PowerSensorI;
extern MPU9250 WingSailAngleSensor;
extern BTStateType BTState;
extern bool SD_Card_Present; // Flag for SD Card Presence
extern WingSailType WingSail;
extern double SteeringServoOutput;

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int DecisionEventValue;
extern int DecisionEventValue2;

extern sim_vessel simulated_vessel;

// Command Line Interpreter - Global Variables
char CLI_Msg[50];
int CLI_i = 0;


// Collect the characters into a command string until end end of line,
// and then process it.
// V1.0 22/12/2015
void CLI_Process_Message(int CommandPort)
{
	// This is called from the medium loop, about 1 sec.
	// Accumulate characters in a command string up to a CR or LF or buffer fills. 
	while ((*Serials[CommandPort]).available())
	{
		//
		char received = (*Serials[CommandPort]).read();
		CLI_Msg[CLI_i++] = received;

		// Process message when new line character is received
		if (received == '\n' || received == '\r' || CLI_i >= sizeof(CLI_Msg) - 2)
		{
			CLI_Msg[CLI_i] = '\0';
			CLI_Processor(CommandPort);
			CLI_i = 0;
		}
	}
}

void CLI_Processor(int CommandPort, String Command)
{
	Command.toCharArray(CLI_Msg, Command.length()); 
	CLI_Processor(CommandPort);
	CLI_i = 0;
}

// Process the Command String.
// Split into command and parameters separated by commas. 
// V1.0 22/12/2015
// V1.1 13/01/2018 added support for a Vessel Command Parameter. i.e. steer a magnetic heading
// V1.2 1/12/2018 changed strcpy to strncpy to guard against corrupting memory with long strings

void CLI_Processor(int CommandPort)
{
	char cmd[4] = "";
	char param1[12] = "";
	char param2[12] = "";
	char param3[12] = "";
	char param4[12] = "";
	char param5[12] = "";
	char param6[12] = "";
	char param7[12] = "";

	strcat(CLI_Msg, ",");

	// Split into command and parameters separated by commas. 
	strncpy(cmd, strtok(CLI_Msg, ","),sizeof(cmd)-1);
	strncpy(param1, strtok(NULL, ","),sizeof(param1)-1);
	strncpy(param2, strtok(NULL, ","),sizeof(param2)-1);
	strncpy(param3, strtok(NULL, ","),sizeof(param3)-1);
	strncpy(param4, strtok(NULL, ","),sizeof(param4)-1);
	strncpy(param5, strtok(NULL, ","),sizeof(param5)-1);
	strncpy(param6, strtok(NULL, ","),sizeof(param6)-1);
	strncpy(param7, strtok(NULL, ","),sizeof(param7)-1);

	// ===============================================
	// Command ech: Echo the command and parameters
	// ===============================================
	if (!strncmp(cmd, "ech",3))
	{
		(*Serials[CommandPort]).print(cmd);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param1);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param2);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param3);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param4);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param5);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param6);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param7);
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// Command mcs: Mission Command Set
	// ===============================================
	// set a Mission Command
	// Parameter 1: Mission Sequence Number 
	// Parameter 2: Mission Command 
	// Parameter 3: MC Param 1
	// Parameter 4: MC Param 2
	// Parameter 5: MC Param 3  
	// Parameter 6: MC Param 4
	// Parameter 7: MC Param 5
	if (!strncmp(cmd, "mcs",3))
	{
		// set the sequence number of the command starting with zero
		int Mission_cmd_ptr = atoi(param1);

		// Advance the mission size to encompass the specified position
		if ((Mission_cmd_ptr + 1) > MissionValues.mission_size) {
			MissionValues.mission_size = Mission_cmd_ptr + 1;
			}

		MissionCommandType mc = MissionCommandType(atoi(param2));
		MissionValues.MissionList[Mission_cmd_ptr].cmd = mc;

		switch (mc)
		{
		case ctGotoWaypoint: 
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lat = atof(param3) * 10000000UL;  //Latitude  * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lng = atof(param4) * 10000000UL;  //Longitude * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param5);      // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			break;

		case ctLoiter:
		case ctLoiterUntil:
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lat = atof(param3) * 10000000UL;  //Latitude  * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].waypoint.lng = atof(param4) * 10000000UL;  //Longitude * 10**7
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param5);	   // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			MissionValues.MissionList[Mission_cmd_ptr].duration = atoi(param7);	   // minutes
			break;

		case ctReturnToHome:
			MissionValues.MissionList[Mission_cmd_ptr].boundary = atoi(param3);	   // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param4);      // Control Mask
			break;

		case ctSteerWindCourse:
			MissionValues.MissionList[Mission_cmd_ptr].SteerAWA = atoi(param3);		// SteerAWA degrees
			MissionValues.MissionList[Mission_cmd_ptr].TrimTabAngle = atoi(param4);  //TrimTabAngle degrees
			MissionValues.MissionList[Mission_cmd_ptr].duration = atoi(param5);      // metres
			MissionValues.MissionList[Mission_cmd_ptr].controlMask = atoi(param6);      // Control Mask
			break;

		default:;
		}
	}

	// ===============================================
	// Command scs: Set Command State
	// ===============================================
	// Set the vessel into a specifed command State
	// Parameter 1: Command State (enumerated Type) 
		//vcsIdle,					// idle state, feathered settings for sails.
		//vcsFullManual,				// The vessel is under manual command via RC
		//vcsPartialManual,				// The vessel is under manual command via RC
		//vcsFollowMission,		// The vessel is under automatic control following the mission list.
		//vcsSteerMagneticCourse,	// The vessel is steering a course relative to the compass
		//vcsSteerWindCourse,		// The vessel is steering a course relative to the wind.
		//vcsReturnToHome,			// return to the preset home location
		//vcsSetHome,				// set home location
		//vcsResetMissionIndex		// Reset the Mission Index, to force a restsart of the mission.		
		//vcsLoiter				// Loiter here.

	if (!strncmp(cmd, "scs",3))
	{
		StateValues.CommandState = VesselCommandStateType(atoi(param1));
		// set both steering angles to the command parameter
		StateValues.SteerCompassBearing = wrap_360(atoi(param2));
		StateValues.SteerWindAngle = atoi(param2);

		// Only set the TrimTabDefaultAngle if the command is vcsSteerWindCourse, otherwise it will interfere normal mission operation
		if (StateValues.CommandState == VesselCommandStateType::vcsSteerWindCourse)
		{
			Configuration.TrimTabDefaultAngle = atoi(param3);
			DecisionEventValue2 = Configuration.TrimTabDefaultAngle;
		}

		// start command timer. Record the start time of each new command	
		MissionValues.MissionCommandStartTime = millis();

		// save the state
		Save_EEPROM_StateValues();

		 DecisionEvent = DecisionEventType::deChangeCommandState ;
		 DecisionEventReason = DecisionEventReasonType::rManualIntervention;
		 DecisionEventValue = atoi(param2);

		// explictly log the commands to set the steering values for wind or compass.
		SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);

		// print the command state
		QueueMessage(TelMessageType::SCS);
	}

	// ===============================================
	// Command scg: Get Command State
	// ===============================================
	// get the current Command State.
	if (!strncmp(cmd, "scg", 3))
	{		
		QueueMessage(TelMessageType::SCG);
	}

	// ===============================================
	// Command mis: Mission Command Index Set
	// ===============================================
	// Set the index to the one less the desired step. i.e. set to zero to commence with step 1.
	// This is usually 0 to start the mission
	// Parameter 1: Mission Sequence Number  
	if (!strncmp(cmd, "mis",3))
	{
		StateValues.mission_index = atoi(param1);

		// save the state
		Save_EEPROM_StateValues();
		
		// since we are forcing a change in the mission index, then force a restart at that index.
		StateValues.StartingMission = true;
		NavData.next_WP_valid = false;

		// update next WP for display purposes.
		set_next_WP_for_display();

		QueueMessage(TelMessageType::MIS);
	}

	// ===============================================
	// Command mig: Mission Command Index Get
	// ===============================================
	// Get the current value of mission command index 
	// No parameters
	if (!strncmp(cmd, "mig",3))
	{
		QueueMessage(TelMessageType::MIG);
	}

	// ===============================================
	// Command mcl: Mission Command List
	// ===============================================
	// return the whole Mission Command list; no parameters needed.
	// No parameters
	if (!strncmp(cmd, "mcl",3))
	{
		(*Serials[CommandPort]).print(F("Mission Command List:"));
		(*Serials[CommandPort]).println();
		char FloatFormatString[16];

		if (MissionValues.mission_size == 0) {
			(*Serials[CommandPort]).println(F("Empty."));
		}

		// loop through the mission list array
		for (int i = 0; i < MissionValues.mission_size; i++)
		{
			(*Serials[CommandPort]).print(i);
			(*Serials[CommandPort]).print(":");

			MissionCommandType mc = MissionValues.MissionList[i].cmd;

			switch (mc)
			{
			case ctGotoWaypoint:
				(*Serials[CommandPort]).print(F("GotoWaypoint:"));
				(*Serials[CommandPort]).print(dtostrf( float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL,10, 5,FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			case ctLoiter:
				(*Serials[CommandPort]).print(F("Loiter:"));
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).print(F(",Duration: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).println();
				break;

			case ctLoiterUntil:
				(*Serials[CommandPort]).print(F("Loiter Until:"));
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(",");
				(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
				(*Serials[CommandPort]).print(F(",Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).print(F(",Time: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).println();
				break;

			case ctReturnToHome:
				(*Serials[CommandPort]).print(F("ReturnToHome,Boundary: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			case ctSteerWindCourse:
				(*Serials[CommandPort]).print(F("SteerWindCourse,AWA:"));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].SteerAWA);
				(*Serials[CommandPort]).print(F(",TrimTabAngle:"));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].TrimTabAngle);
				(*Serials[CommandPort]).print(F(",Duration: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
				(*Serials[CommandPort]).print(F(",Control: "));
				(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
				(*Serials[CommandPort]).println();
				break;

			default:
				(*Serials[CommandPort]).print(F("Unknown command"));
			}
		}
	}

	// ===============================================
	// Command mcp: Mission Command List for Plotting
	// ===============================================
	// return the whole Mission Command list; no parameters needed.
	// No parameters
	if (!strncmp(cmd, "mcp", 3))
	{
		QueueMessage(TelMessageType::MCP);
	}

	// ===============================================
	// Command mcc:  Mission Command List - Clear All
	// ===============================================
	// clear the Mission Command List
	// No parameters
	if (!strncmp(cmd, "mcc",3))
	{
		// reset all mission values and flags
		MissionValues.mission_size = 0;
		StateValues.mission_index = 0;
		StateValues.StartingMission = true;
		NavData.next_WP_valid = false;

		QueueMessage(TelMessageType::MCC);
	}

	// ===============================================
	// Command lcs: Set Current Location, and disable GPS. 
	// i.e. use a simulated GPS. 
	// ===============================================
	// Set Current Location
    // Parameter 1: simulated Lat 
	// Parameter 2: simulated Lon 
	if (!strncmp(cmd, "lcs", 3))
	{
		UseSimulatedVessel = true;
		
		// set the Current location
		//NavData.Currentloc.lat = atof(param1) * 10000000UL;  //Latitude  * 10**7
		//NavData.Currentloc.lng = atof(param2) * 10000000UL;  //Longitude * 10**7

		simulated_vessel.Currentloc.lat = atof(param1) * 10000000UL;  //Latitude  * 10**7
		simulated_vessel.Currentloc.lng = atof(param2) * 10000000UL;  //Longitude * 10**7
		simulated_vessel.Heading = atoi(param3);
	}

	// ===============================================
	// Command HLS, Set Home Location
	// ===============================================
	// Parameter 1: Range from current location - metres
	// Parameter 2: True Bearing from current location - degrees
	// 
	if (!strncmp(cmd, "hls", 3))
	{
		// get the current location 
		StateValues.home = NavData.Currentloc;

		float distance = atof(param1); // metres
		float bearing = atof(param2);  // degrees

		location_update(StateValues.home, bearing, distance);
		StateValues.home_is_set = true;
		
		// save the state
		Save_EEPROM_StateValues();

		QueueMessage(TelMessageType::HLG);
	}

	// ===============================================
	// Command HLG, Get Home Location
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "hlg", 3))
	{
		QueueMessage(TelMessageType::HLG);
	}

	// ===============================================
	// Command HLC, Clear Home Location
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "hlc", 3))
	{
		// Clear the flag
		StateValues.home_is_set = false;

		char MsgString[16];

		(*Serials[CommandPort]).print("HOM:");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lat) / 10000000UL, 10, 5, MsgString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lng) / 10000000UL, 10, 5, MsgString));
		(*Serials[CommandPort]).print(",");
		StateValues.home_is_set ? (*Serials[CommandPort]).print("Set:true") : (*Serials[CommandPort]).print("Set:false");
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// Command ver, Get Software Version
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "ver", 3))
	{
		QueueMessage(TelMessageType::VER);
	}

	// ===============================================
	// Command TMG, Get Current Time
	// ===============================================
	// No parameters
	if (!strncmp(cmd, "tmg", 3))
	{
		QueueMessage(TelMessageType::TMG);
	}

	// ===============================================
	// Command log, Set Logging Level on the SD Card
	// ===============================================
	// Parameter 1: Logging Level: 0,1
	// 
	if (!strncmp(cmd, "log", 3))
	{
		Configuration.SDCardLoggingMask = atol(param1);
		QueueMessage(TelMessageType::LOG);
	}

	// ===============================================
	// Command lcd, Set LCD Logging Level.
	// ===============================================
	// Parameter 1: Logging Level: 0,1,2,3
	// 
	if (!strncmp(cmd, "lcd", 3))
	{
		Configuration.DisplayScreenView = *param1;
		QueueMessage(TelMessageType::LCD);
	}

	// ===============================================
	// Command eqg, Get Equipment Status
	// ===============================================
	//  No parameters
	// 
	// Return the status of the Wing Angle Sensor, SD Card.

	if (!strncmp(cmd, "eqg", 3))
	{
		QueueMessage(TelMessageType::EQG);
	}

	// ===============================================
	// Command ccs,  Compass calibration Save
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "ccs", 3))
	{
		QueueMessage(TelMessageType::CCS);

		IMU_Save_Cal();
	}

	// ===============================================
	// Command ccg,  Compass calibration Get Status
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "ccg", 3))
	{
		QueueMessage(TelMessageType::CCG);
	}

	// ===============================================
	// Command wc1,  Wingsail Angle Sensor calibration ON
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "wc1", 3))
	{
		QueueMessage(TelMessageType::WC1);

		// enable calibration mode. Fast reads and maintain enable setting of the min/max calibration values
		WingSailAngleSensor.MagneticAngleCalibrationMode = true;

		// init the Calibration limits
		IMU_Mag_Init();
	}

	// ===============================================
	// Command wc0,  Wingsail Angle Sensor calibration OFF
	// ===============================================
	//  No parameters
	// 
	if (!strncmp(cmd, "wc0", 3))
	{
		QueueMessage(TelMessageType::WC0);

		// disable calibation mode
		WingSailAngleSensor.MagneticAngleCalibrationMode = false;

		// save the Calibration values
		Save_EEPROM_ConfigValues();
	}

	// ===============================================
	// Command wsc,  Wingsail Command
	// ===============================================
	//  two parameters
	// 
	if (!strncmp(cmd, "wsc", 3))
	{
		if (BTState == BTStateType::Connected)
		{
			(*Serials[Configuration.BluetoothPort]).print(param1);
			(*Serials[Configuration.BluetoothPort]).print(",");
			(*Serials[Configuration.BluetoothPort]).print(param2);
			(*Serials[Configuration.BluetoothPort]).println();
		}
		else
		{
			(*Serials[CommandPort]).print(F("MSG,BT "));
			(*Serials[CommandPort]).println(GetBTStatus(BTState));
		}
	}

	// ===============================================
	// Command sav, Save Data to EEPROM
	// ===============================================
	//  Parameter 1: Entity to be saved: m-Mission, c-Config, s-State , y-Save State-On, n-Save-State-Off
	// 
	if (!strncmp(cmd, "sav", 3)) 
	{
		(*Serials[CommandPort]).print(F("SAV, Save to EEPROM: "));

		switch (*param1)
		{
		case 'm':
			(*Serials[CommandPort]).print(F("Mission"));
			Save_EEPROM_Mission();
			break;

		case 'c':
			(*Serials[CommandPort]).print(F("Configuration"));
			Save_EEPROM_ConfigValues();
			break;

		case 's':
			(*Serials[CommandPort]).print(F("State"));
			Save_EEPROM_StateValues();
			break;

		case 'y':
			(*Serials[CommandPort]).print(F("Save State-Yes"));
			Configuration.SaveStateValues = true;
			break;

		case 'n':
			(*Serials[CommandPort]).print(F("Save State-No"));
			Configuration.SaveStateValues = false;
			break;

		default:
			(*Serials[CommandPort]).print(F("unknown"));
		}
		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// 	Command dsp, Display messages on LCD/OLED
	// ===============================================
	// Paramters: Line1, Line2
	// 
	if (!strncmp(cmd, "dsp", 3))
	{
		strcpy(MessageDisplayLine1, param1);
		strcpy(MessageDisplayLine2, param2);
	}

	// ===============================================
	// 	prl, Parameter List   
	// ===============================================
	// Parameters: none.
	// this initiates listing all parameters
	if (!strncmp(cmd, "prl", 3))
	{
		QueueMessage(TelMessageType::PRL);
	}

	// ===============================================
	// 	prg, Parameter Get   
	// ===============================================
	// Parameters: Parameter Index Number
	// 
	if (!strncmp(cmd, "prg", 3))
	{
		int ParameterIndex = atoi(param1);
		ListParameter(CommandPort,ParameterIndex);
	}

	// ===============================================
	// 	prs, Parameter Set   
	// ===============================================
	// Parameters: Parameter Index Number, Parameter Value
	// 
	if (!strncmp(cmd, "prs", 3))
	{
		char MsgString[16];

		int ParameterIndex = atoi(param1);

		// log the change to the SD Card.
		SD_Logging_Event_ParameterChange(ParameterIndex, param2);

		(*Serials[CommandPort]).print(F("SET,"));
		(*Serials[CommandPort]).print(ParameterIndex);
		(*Serials[CommandPort]).print(",");

		switch (ParameterIndex)
		{
		case 1:
			Configuration.CompassOffsetAngle = atof(param2);
			(*Serials[CommandPort]).print(F("Mag_Offset,"));
			(*Serials[CommandPort]).print(Configuration.CompassOffsetAngle);
			break;

		case 2:
			Configuration.mx_min = atof(param2);
			(*Serials[CommandPort]).print(F("Wingsail_mx_min,"));
			(*Serials[CommandPort]).print(Configuration.mx_min);
			break;

		case 3:
			Configuration.mx_max = atof(param2);
			(*Serials[CommandPort]).print(F("Wingsail_mx_max,"));
			(*Serials[CommandPort]).print(Configuration.mx_max);
			break;

		case 4:
			Configuration.my_min = atof(param2);
			(*Serials[CommandPort]).print(F("Wingsail_my_min,"));
			(*Serials[CommandPort]).print(Configuration.my_min);
			break;

		case 5:
			Configuration.my_max = atof(param2);
			(*Serials[CommandPort]).print(F("Wingsail_my_max,"));
			(*Serials[CommandPort]).print(Configuration.my_max);
			break;

		case 6:
			Configuration.LoiterRadius = atol(param2);
			(*Serials[CommandPort]).print(F("LoiterRadius,"));
			(*Serials[CommandPort]).print(Configuration.LoiterRadius);
			break;

		case 7:
			Configuration.timezone_offset = atoi(param2);
			(*Serials[CommandPort]).print(F("timezone_offset,"));
			(*Serials[CommandPort]).print(Configuration.timezone_offset);
			break;

		case 8:
			Configuration.DisplayScreenView = *param2;
			(*Serials[CommandPort]).print(F("LCDScreenView,"));
			(*Serials[CommandPort]).print(Configuration.DisplayScreenView);
			break;

		case 9:
			Configuration.SDCardLoggingMask = atol(param2);
			(*Serials[CommandPort]).print(F("SDCardLoggingMask,"));
			(*Serials[CommandPort]).print(Configuration.SDCardLoggingMask);
			break;

		case 10:
			Configuration.TelemetryLoggingMask = atol(param2);
			(*Serials[CommandPort]).print(F("TelemetryLoggingMask,"));
			(*Serials[CommandPort]).print(Configuration.TelemetryLoggingMask);
			break;

		case 11:
			Configuration.SaveStateValues = atoi(param2);
			(*Serials[CommandPort]).print(F("SaveStateValues,"));
			Configuration.SaveStateValues ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
			break;

		case 12:
			Configuration.MinimumAngleUpWind = atoi(param2);
			(*Serials[CommandPort]).print(F("MinimumAngleUpWind,"));
			(*Serials[CommandPort]).print(Configuration.MinimumAngleUpWind);
			break;

		case 13:
			Configuration.MinimumAngleDownWind = atoi(param2);
			(*Serials[CommandPort]).print(F("MinimumAngleDownWind,"));
			(*Serials[CommandPort]).print(Configuration.MinimumAngleDownWind);
			break;

		case 14:
			Configuration.WindAngleCalibrationOffset = atoi(param2);
			(*Serials[CommandPort]).print(F("WindAngleCalibrationOffset,"));
			(*Serials[CommandPort]).print(Configuration.WindAngleCalibrationOffset);
			break;

		case 15:
			Configuration.WPCourseHoldRadius = atoi(param2);
			(*Serials[CommandPort]).print(F("WPCourseHoldRadius,"));
			(*Serials[CommandPort]).print(Configuration.WPCourseHoldRadius);
			break;

		case 16:
			if (!strncmp(param2, "t", 1)) { Configuration.SDCardLogDelimiter = '\t';}
			if (!strncmp(param2, ",", 1)) { Configuration.SDCardLogDelimiter = ',';}

			(*Serials[CommandPort]).print(F("SDCardDelimiter,"));

			if (!strncmp(param2, "t", 1)) { (*Serials[CommandPort]).print(F("Tab"));}
			if (!strncmp(param2, ",", 1)) { (*Serials[CommandPort]).print(",");}
			break;

		case 17:
			Configuration.RTHTimeManualControl = atoi(param2);
			(*Serials[CommandPort]).print(F("RTHTimeManualControl,"));
			(*Serials[CommandPort]).print(Configuration.RTHTimeManualControl);
			break;

		case 18:
			Configuration.MinimumWindChangeTime = atoi(param2);
			(*Serials[CommandPort]).print(F("MinimumWindChangeTime,"));
			(*Serials[CommandPort]).print(Configuration.MinimumWindChangeTime);
			break;

		case 19:
			Configuration.TargetHeadingFilterConstant = atof(param2);
			(*Serials[CommandPort]).print(F("TargetHeadingFilterConstant,"));
			(*Serials[CommandPort]).print(dtostrf(Configuration.TargetHeadingFilterConstant, 10, 5, MsgString));
			break;

		case 20:
			Configuration.MaxFileSize = atoi(param2);
			(*Serials[CommandPort]).print(F("MaxFileSize,"));
			(*Serials[CommandPort]).print(Configuration.MaxFileSize);
			break;

		case 21:
			Configuration.RC_FailSafe_Ch0 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_FailSafe_Ch0,"));
			(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch0);
			break;

		case 22:
			Configuration.RC_FailSafe_Ch1 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_FailSafe_Ch1,"));
			(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch1);
			break;

		case 23:
			Configuration.RC_FailSafe_Ch2 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_FailSafe_Ch2,"));
			(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch2);
			break;

		case 24:
			Configuration.RC_FailSafe_Ch3 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_FailSafe_Ch3,"));
			(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch3);
			break;

		case 25:
			Configuration.RC_FailSafe_Time = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_FailSafe_Time,"));
			(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Time);
			break;

		case 26:
			Configuration.RC_IN_Channel_Command = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Channel_Command,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Command);
			break;

		case 27:
			Configuration.RC_IN_Channel_Steering = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Channel_Steering,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Steering);
			break;

		case 28:
			Configuration.RC_IN_Channel_Sail = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Channel_Sail,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Sail);
			break;

		case 29:
			Configuration.Servo_Channel_Steering = atoi(param2);
			(*Serials[CommandPort]).print(F("Servo_Channel_Steering,"));
			(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering);
			break;

		case 30:
			Configuration.Servo_Channel_Steering_Stbd = atoi(param2);
			(*Serials[CommandPort]).print(F("Servo_Channel_Steering_Stbd,"));
			(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering_Stbd);
			break;

		case 31:
			Configuration.Servo_Channel_Sail = atoi(param2);
			(*Serials[CommandPort]).print(F("Servo_Channel_Sail,"));
			(*Serials[CommandPort]).print(Configuration.Servo_Channel_Sail);
			break;

		case 32:
			Configuration.RC_IN_Command_Threhold0 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold0,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold0);
			break;

		case 33:
			Configuration.RC_IN_Command_Threhold1 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold1,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold1);
			break;

		case 34:
			Configuration.RC_IN_Command_Threhold2 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold2,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold2);
			break;

		case 35:
			Configuration.RC_IN_Command_Threhold3 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold3,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold3);
			break;

		case 36:
			Configuration.RC_IN_Command_Threhold4 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold4,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold4);
			break;

		case 37:
			Configuration.RC_IN_Command_Threhold9 = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold9,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold9);
			break;

		case 38:
			Configuration.DualRudder = atoi(param2);
			(*Serials[CommandPort]).print(F("DualRudder,"));
			Configuration.DualRudder ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
			break;

		case 39:
			Configuration.CommandPort = atoi(param2);
			(*Serials[CommandPort]).print(F("CommandPort,"));
			(*Serials[CommandPort]).print(Configuration.CommandPort);
			break;

		case 40:
			Configuration.BluetoothPort = atoi(param2);
			(*Serials[CommandPort]).print(F("BluetoothPort,"));
			(*Serials[CommandPort]).print(Configuration.BluetoothPort);
			break;

		case 41:
			Configuration.SatCommsPort = atoi(param2);
			(*Serials[CommandPort]).print(F("SatCommsPort,"));
			(*Serials[CommandPort]).print(Configuration.SatCommsPort);
			break;

		case 42:
			Configuration.CommandPortBaudRate = atol(param2);
			(*Serials[CommandPort]).print(F("CommandPortBaudRate,"));
			(*Serials[CommandPort]).print(Configuration.CommandPortBaudRate);
			break;

		case 43:
			Configuration.BTPortBaudRate = atol(param2);
			(*Serials[CommandPort]).print(F("BTPortBaudRate,"));
			(*Serials[CommandPort]).print(Configuration.BTPortBaudRate);
			break;

		case 44:
			Configuration.SatCommsPortBaudRate = atol(param2);
			(*Serials[CommandPort]).print(F("SatCommsPortBaudRate,"));
			(*Serials[CommandPort]).print(Configuration.SatCommsPortBaudRate);
			break;

		case 45:
			Configuration.MagnetVariation = atof(param2);
			(*Serials[CommandPort]).print(F("MagnetVariation,"));
			(*Serials[CommandPort]).print(Configuration.MagnetVariation);
			break;

		case 46:
			Configuration.pidDirection = atoi(param2);
			(*Serials[CommandPort]).print(F("pidDirection,"));
			if (Configuration.pidDirection == 0) {
				(*Serials[CommandPort]).print("Direct");
			}
			else {
				(*Serials[CommandPort]).print("Reverse");
			}
			SteeringPID_Init(); // update PID immediately
			break;

		case 47:
			Configuration.pidKp = atol(param2);
			(*Serials[CommandPort]).print(F("pidKp,"));
			(*Serials[CommandPort]).print(Configuration.pidKp);
			SteeringPID_Init(); // update PID immediately
			break;

		case 48:
			Configuration.pidKi = atol(param2);
			(*Serials[CommandPort]).print(F("pidKi,"));
			(*Serials[CommandPort]).print(Configuration.pidKi);
			SteeringPID_Init(); // update PID immediately
			break;

		case 49:
			Configuration.pidKd = atol(param2);
			(*Serials[CommandPort]).print(F("pidKd,"));
			(*Serials[CommandPort]).print(Configuration.pidKd);
			SteeringPID_Init(); // update PID immediately
			break;

		case 50:
			Configuration.pidOutputmin = atol(param2);
			(*Serials[CommandPort]).print(F("pidOutputmin,"));
			(*Serials[CommandPort]).print(Configuration.pidOutputmin);
			SteeringPID_Init(); // update PID immediately
			break;

		case 51:
			Configuration.pidOutputmax = atol(param2);
			(*Serials[CommandPort]).print(F("pidOutputmax,"));
			(*Serials[CommandPort]).print(Configuration.pidOutputmax);
			SteeringPID_Init(); // update PID immediately
			break;

		case 52:
			Configuration.pidCentre = atol(param2);
			(*Serials[CommandPort]).print(F("pidCentre,"));
			(*Serials[CommandPort]).print(Configuration.pidCentre);
			break;

		case 53:
			Configuration.RC_IN_Channel_Motor = atoi(param2);
			(*Serials[CommandPort]).print(F("RC_IN_Channel_Motor,"));
			(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Motor);
			break;

		case 54:
			Configuration.Servo_Channel_Motor = atoi(param2);
			(*Serials[CommandPort]).print(F("Servo_Channel_Motor,"));
			(*Serials[CommandPort]).print(Configuration.Servo_Channel_Motor);
			break;

		case 55:
			Configuration.UseMotor = atoi(param2);
			(*Serials[CommandPort]).print(F("UseMotor,"));
			Configuration.UseMotor ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
			break;

		case 56:
			Configuration.FailsafeCommandState = (VesselCommandStateType)atoi(param2);
			(*Serials[CommandPort]).print(F("FailsafeCommandState,"));
			(*Serials[CommandPort]).print(Configuration.FailsafeCommandState);
			break;

		case 57:
			Configuration.TrimTabDefaultAngle = atoi(param2);
			(*Serials[CommandPort]).print(F("TrimTab Default Angle,"));
			(*Serials[CommandPort]).print(Configuration.TrimTabDefaultAngle);
			break;

		case 58:
			Configuration.TWD_Offset = atoi(param2);
			(*Serials[CommandPort]).print(F("AWA to TWD_Offset Angle,"));
			(*Serials[CommandPort]).print(Configuration.TWD_Offset);
			break;

		case 59:
			Configuration.SailableAngleMargin = atoi(param2);
			(*Serials[CommandPort]).print(F("Sailable Angle Margin,"));
			(*Serials[CommandPort]).print(Configuration.SailableAngleMargin);
			break;

		case 60:
			//	spare 60
			break;

		case 61:
			//	spare 61
			break;

		case 62:
			//	spare 62
			break;

		case 63:
			//	spare 63
			break;

		case 64:
			//	spare 64
			break;

		case 65:
			//	spare 65
			break;

		case 66:
			Configuration.TrimTabScale = atoi(param2);
			(*Serials[CommandPort]).print(F("TrimTabScale,"));
			(*Serials[CommandPort]).print(Configuration.TrimTabScale);
			break;

		case 67:
			Configuration.TrimTabOffset = atoi(param2);
			(*Serials[CommandPort]).print(F("TrimTabOffset,"));
			(*Serials[CommandPort]).print(Configuration.TrimTabOffset);
			break;

		default:
			(*Serials[CommandPort]).print(F("Unknown"));
		}

		(*Serials[CommandPort]).println();
	}

	// ===============================================
	// Command pwr: Set power on TEL or RC
	// ===============================================
	// Parameter 1: TEL or RC
	// Parameter 2: Power On or Off (on = 1, off = 0)

	if (!strncmp(cmd, "pwr", 3))
	{
		boolean PowerOn = (byte) atoi(param2);

		if (!strncmp(param1, "tel", 3)) 
		{
			TelemetryPowerOn(PowerOn);
			(*Serials[CommandPort]).print(F("PWR,TEL,"));
			(*Serials[CommandPort]).println(PowerOn);
		}

		if (!strncmp(param1, "rc", 2))
		{
			RCPowerOn(PowerOn);
			(*Serials[CommandPort]).print(F("PWR,RC,"));
			(*Serials[CommandPort]).println(PowerOn);
		}
	}

	// ********** Start - LoRa Polling Commands *****************************************************************************
	
	// ===============================================
	// Command: LAP - (LoRa) Nav - 	responses: CTE, DTW, BTW, CDA, LAT, LON, COG, SOG, HDG
	// ===============================================
	// No Parameters

	if (!strncmp(cmd, "lna", 3))
	{
		QueueMessage(TelMessageType::LNA);
	}

	// ===============================================
	// Command: LAT - (LoRa) Attitude:	responses: HDG, Pitch, Roll, Roll_avg
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lat", 3))
	{
		QueueMessage(TelMessageType::LAT);
	}

	// ===============================================
	// Command: LPO - (LoRa) Power:	responses: Voyager Power V,I, Wingsail Power V, I
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lpo", 3))
	{
		QueueMessage(TelMessageType::LPO);
	}

	// ===============================================
	// Command: LWP - (LoRa) Waypoint:	responses: Prev WP, Next WP, Max CTE
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lwp", 3))
	{
		QueueMessage(TelMessageType::LWP);
	}

	// ===============================================
	// Command: LMI - (LoRa) current Mission Step Responses: MI, Cmd, Duration, SteerAWA, TTA
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lmi", 3))
	{
		QueueMessage(TelMessageType::LMI);
	}

	// ===============================================
	// Command: LWI - (LoRa) Wind data: Responses AWA / TWD,  TWS, WA / TTA
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lwi", 3))
	{
		QueueMessage(TelMessageType::LWI);

	}

	// ===============================================
	// Command: LVS - (LoRa) GetVessel State - Response: Vessel State
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lvs", 3))
	{
		QueueMessage(TelMessageType::LVS);
	}

	// ===============================================
	// Command: LPF - (LoRa) Get Sailing Performance 
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lpf", 3))
	{
		QueueMessage(TelMessageType::LPF);
	}

	// ===============================================
	// Command: LSV - (LoRa) Servo positions - Response: Rudder us, Trim Tab us
	// ===============================================
	// No Parameters
	if (!strncmp(cmd, "lsv", 3))
	{
		QueueMessage(TelMessageType::LSV);
	}
	// ********** End - LoRa Polliing Commands *****************************************************************************


}

void ListParameter(int CommandPort,int ParameterIndex)
{
	// function to list the value of a single parameter specified by its parameter number
	// V1.1 11/10/2016 added MaxFileSize.

	char MsgString[16];
	
	(*Serials[CommandPort]).print(F("VAR,"));
	(*Serials[CommandPort]).print(ParameterIndex);
	(*Serials[CommandPort]).print(",");

	switch (ParameterIndex)
	{
	case 1:
		(*Serials[CommandPort]).print(F("Mag_Offset,"));
		(*Serials[CommandPort]).print(Configuration.CompassOffsetAngle);
		break;

	case 2:
		(*Serials[CommandPort]).print(F("Wingsail_mx_min,"));
		(*Serials[CommandPort]).print(Configuration.mx_min);
		break;

	case 3:
		(*Serials[CommandPort]).print(F("Wingsail_mx_max,"));
		(*Serials[CommandPort]).print(Configuration.mx_max);
		break;

	case 4:
		(*Serials[CommandPort]).print(F("Wingsail_my_min,"));
		(*Serials[CommandPort]).print(Configuration.my_min);
		break;

	case 5:
		(*Serials[CommandPort]).print(F("Wingsail_my_max,"));
		(*Serials[CommandPort]).print(Configuration.my_max);
		break;

	case 6:
		(*Serials[CommandPort]).print(F("LoiterRadius,"));
		(*Serials[CommandPort]).print(Configuration.LoiterRadius);
		break;

	case 7:
		(*Serials[CommandPort]).print(F("timezone_offset,"));
		(*Serials[CommandPort]).print(Configuration.timezone_offset);
		break;

	case 8:
		(*Serials[CommandPort]).print(F("LCDScreenView,"));
		(*Serials[CommandPort]).print(Configuration.DisplayScreenView);
		break;

	case 9:
		(*Serials[CommandPort]).print(F("SDCardLoggingMask,"));
		(*Serials[CommandPort]).print(Configuration.SDCardLoggingMask);
		break;

	case 10:
		(*Serials[CommandPort]).print(F("TelemetryLoggingMask,"));
		(*Serials[CommandPort]).print(Configuration.TelemetryLoggingMask);
		break;

	case 11:
		(*Serials[CommandPort]).print(F("SaveStateValues,"));
		Configuration.SaveStateValues ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 12:
		(*Serials[CommandPort]).print(F("MinimumAngleUpWind,"));
		(*Serials[CommandPort]).print(Configuration.MinimumAngleUpWind);
		break;

	case 13:
		(*Serials[CommandPort]).print(F("MinimumAngleDownWind,"));
		(*Serials[CommandPort]).print(Configuration.MinimumAngleDownWind);
		break;

	case 14:
		(*Serials[CommandPort]).print(F("WindAngleCalibrationOffset,"));
		(*Serials[CommandPort]).print(Configuration.WindAngleCalibrationOffset);
		break;

	case 15:
		(*Serials[CommandPort]).print(F("WPCourseHoldRadius,"));
		(*Serials[CommandPort]).print(Configuration.WPCourseHoldRadius);
		break;

	case 16:
		(*Serials[CommandPort]).print(F("SDCardDelimiter,"));

		if (Configuration.SDCardLogDelimiter == '\t') { (*Serials[CommandPort]).print(F("Tab")); }
		if (Configuration.SDCardLogDelimiter == ',') { (*Serials[CommandPort]).print(","); }
		break;

	case 17:
		(*Serials[CommandPort]).print(F("RTHTimeManualControl,"));
		(*Serials[CommandPort]).print(Configuration.RTHTimeManualControl);
		break;

	case 18:
		(*Serials[CommandPort]).print(F("MinimumWindChangeTime,"));
		(*Serials[CommandPort]).print(Configuration.MinimumWindChangeTime);
		break;

	case 19:
		(*Serials[CommandPort]).print(F("TargetHeadingFilterConstant,"));
		(*Serials[CommandPort]).print(Configuration.TargetHeadingFilterConstant);
		break;

	case 20:
		(*Serials[CommandPort]).print(F("MaxFileSize,"));
		(*Serials[CommandPort]).print(Configuration.MaxFileSize);
		break;

	case 21:
		(*Serials[CommandPort]).print(F("RC_FailSafe_Ch0,"));
		(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch0);
		break;

	case 22:
		(*Serials[CommandPort]).print(F("RC_FailSafe_Ch1,"));
		(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch1);
		break;

	case 23:
		(*Serials[CommandPort]).print(F("RC_FailSafe_Ch2,"));
		(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch2);
		break;

	case 24:
		(*Serials[CommandPort]).print(F("RC_FailSafe_Ch3,"));
		(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Ch3);
		break;

	case 25:
		(*Serials[CommandPort]).print(F("RC_FailSafe_Time,"));
		(*Serials[CommandPort]).print(Configuration.RC_FailSafe_Time);
		break;

	case 26:
		(*Serials[CommandPort]).print(F("RC_IN_Channel_Command,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Command);
		break;

	case 27:
		(*Serials[CommandPort]).print(F("RC_IN_Channel_Steering,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Steering);
		break;

	case 28:
		(*Serials[CommandPort]).print(F("RC_IN_Channel_Sail,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Sail);
		break;

	case 29:
		(*Serials[CommandPort]).print(F("Servo_Channel_Steering,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering);
		break;

	case 30:
		(*Serials[CommandPort]).print(F("Servo_Channel_Steering_Stbd,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Steering_Stbd);
		break;

	case 31:
		(*Serials[CommandPort]).print(F("Servo_Channel_Sail,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Sail);
		break;

	case 32:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold0,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold0);
		break;

	case 33:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold1,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold1);
		break;

	case 34:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold2,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold2);
		break;

	case 35:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold3,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold3);
		break;

	case 36:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold4,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold4);
		break;

	case 37:
		(*Serials[CommandPort]).print(F("RC_IN_Command_Threhold9,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Command_Threhold9);
		break;

	case 38:
		(*Serials[CommandPort]).print(F("DualRudder,"));
		Configuration.DualRudder ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 39:
		(*Serials[CommandPort]).print(F("CommandPort,"));
		(*Serials[CommandPort]).print(Configuration.CommandPort);
		break;

	case 40:
		(*Serials[CommandPort]).print(F("BluetoothPort,"));
		(*Serials[CommandPort]).print(Configuration.BluetoothPort);
		break;

	case 41:
		(*Serials[CommandPort]).print(F("SatCommsPort,"));
		(*Serials[CommandPort]).print(Configuration.SatCommsPort);
		break;

	case 42:
		(*Serials[CommandPort]).print(F("CommandPortBaudRate,"));
		(*Serials[CommandPort]).print(Configuration.CommandPortBaudRate);
		break;

	case 43:
		(*Serials[CommandPort]).print(F("BTPortBaudRate,"));
		(*Serials[CommandPort]).print(Configuration.BTPortBaudRate);
		break;

	case 44:
		(*Serials[CommandPort]).print(F("SatCommsPortBaudRate,"));
		(*Serials[CommandPort]).print(Configuration.SatCommsPortBaudRate);
		break;

	case 45:
		(*Serials[CommandPort]).print(F("MagnetVariation,"));
		(*Serials[CommandPort]).print(Configuration.MagnetVariation);
		break;

	case 46:
		(*Serials[CommandPort]).print(F("pidDirection,"));
		if (Configuration.pidDirection == 0) {
			(*Serials[CommandPort]).print("Direct");
		} else{
			(*Serials[CommandPort]).print("Reverse");
		}
		break;

	case 47:
		(*Serials[CommandPort]).print(F("pidKp,"));
		(*Serials[CommandPort]).print(Configuration.pidKp);
		break;

	case 48:
		(*Serials[CommandPort]).print(F("pidKi,"));
		(*Serials[CommandPort]).print(Configuration.pidKi);
		break;

	case 49:
		(*Serials[CommandPort]).print(F("pidKd,"));
		(*Serials[CommandPort]).print(Configuration.pidKd);
		break;

	case 50:
		(*Serials[CommandPort]).print(F("pidOutputmin,"));
		(*Serials[CommandPort]).print(Configuration.pidOutputmin);
		break;

	case 51:
		(*Serials[CommandPort]).print(F("pidOutputmax,"));
		(*Serials[CommandPort]).print(Configuration.pidOutputmax);
		break;

	case 52:
		(*Serials[CommandPort]).print(F("pidCentre,"));
		(*Serials[CommandPort]).print(Configuration.pidCentre);
		break;

	case 53:
		(*Serials[CommandPort]).print(F("RC_IN_Channel_Motor,"));
		(*Serials[CommandPort]).print(Configuration.RC_IN_Channel_Motor);
		break;

	case 54:
		(*Serials[CommandPort]).print(F("Servo_Channel_Motor,"));
		(*Serials[CommandPort]).print(Configuration.Servo_Channel_Motor);
		break;

	case 55:
		(*Serials[CommandPort]).print(F("UseMotor,"));
		Configuration.UseMotor ? (*Serials[CommandPort]).print("true") : (*Serials[CommandPort]).print("false");
		break;

	case 56:
		(*Serials[CommandPort]).print(CommandStateToString(Configuration.FailsafeCommandState));
		break;

	case 57:
		(*Serials[CommandPort]).print(F("TrimTab Default Angle,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabDefaultAngle);
		break;

	case 58:
		(*Serials[CommandPort]).print(F("AWA to TWD_Offset Angle,"));
		(*Serials[CommandPort]).print(Configuration.TWD_Offset);
		break;

	case 59:
		(*Serials[CommandPort]).print(F("Sailable Angle Margin,"));
		(*Serials[CommandPort]).print(Configuration.SailableAngleMargin);
		break;

	case 60:
		(*Serials[CommandPort]).print(F("Spare 60,"));
		break;

	case 61:
		(*Serials[CommandPort]).print(F("Spare 61,"));
		break;

	case 62:
		(*Serials[CommandPort]).print(F("Spare 62,"));
		break;

	case 63:
		(*Serials[CommandPort]).print(F("Spare 63,"));
		break;

	case 64:
		(*Serials[CommandPort]).print(F("Spare 64,"));
		break;

	case 65:
		(*Serials[CommandPort]).print(F("Spare 65,"));
		break;

	case 66:
		(*Serials[CommandPort]).print(F("TrimTabScale,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabScale);
		break;

	case 67:
		(*Serials[CommandPort]).print(F("TrimTabOffset,"));
		(*Serials[CommandPort]).print(Configuration.TrimTabOffset);
		break;

	default:
		(*Serials[CommandPort]).print(F("Unknown"));
	}

	(*Serials[CommandPort]).println();
}

void ShowCommandState(int CommandPort, VesselCommandStateType cs)
{
	// function to print a string representation for the Vessel Command state
	// V1.1 12/1/2018 added csLoiter
	switch (cs)
	{
	case vcsIdle:
		(*Serials[CommandPort]).print(F("Idle"));
		break;

	case vcsFullManual:
		(*Serials[CommandPort]).print(F("Full Manual"));
		break;

	case vcsPartialManual:
		(*Serials[CommandPort]).print(F("Part Manual"));
		break;

	case vcsFollowMission:
		(*Serials[CommandPort]).print(F("FollowMission"));
		break;

	case vcsSteerMagneticCourse:
		(*Serials[CommandPort]).print(F("SteerMagneticCourse"));
		break;

	case vcsSteerWindCourse:
		(*Serials[CommandPort]).print(F("SteerWindCourse"));
		break;

	case vcsReturnToHome:
		(*Serials[CommandPort]).print(F("ReturnToHome"));
		break;

	case vcsSetHome:
		(*Serials[CommandPort]).print(F("SetHome"));
		break;

	case vcsResetMissionIndex:
		(*Serials[CommandPort]).print(F("ResetMissionIndex"));
		break;

	case vcsLoiter:
		(*Serials[CommandPort]).print(F("Loiter Here"));
		break;

	default:
		(*Serials[CommandPort]).print(F("unknown"));
	}

}