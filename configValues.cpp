// Manage Configuration values stored in the EEPROM.
// This covers the storage structures, and also checks the validity of the stored sturcture versus the expect structure.
// 
// V1.1 11/10/2016 added MaxFileSize.
// V1.2 31/10/2016 added MagnetVariation
// V1.3 13/11/2016 added Steering PID values to Config
// V1.4 6/12/2017 added Motor control parameters
// V1.5 17/1/2018 added default compass calibration values
// V1.6 2/4/2018 added Voltage Measurement parameters.
// V1.7 31/10/2018 added TrimTabScale Scale and Offset


#include "configValues.h"
#include <EEPROM.h>
#include "eepromAnything.h"
#include "Mission.h"
#include "CLI.h"
#include "CommandState_Processor.h"
#include "PID_v1.h"
#include "WearTracking.h"

extern configValuesType Configuration;
extern MissionCommand MissionList[MaxMissionCommands];
extern MissionValuesStruct MissionValues;
extern StateValuesStruct StateValues;
extern HardwareSerial *Serials[];

extern WearCounter PortRudderUsage;
extern WearCounter StarboardRudderUsage;
extern WearCounter TrimTabUsage;
extern VesselUsageCountersStruct VesselUsageCounters;

// Calculate the base address of each structure in the EEPROM.
// This done by getting the size of the previous object and adding it to the address of the previous object.
static const int VesselUsageCountersAddress = 0;
static const int sizeof_VesselUsageCounters = sizeof(VesselUsageCounters);

static const int ConfigurationAddress = sizeof_VesselUsageCounters + VesselUsageCountersAddress;
static const int sizeof_Configuration = sizeof(Configuration);

static const int MissionValuesAddress = sizeof_Configuration + ConfigurationAddress;
static const int sizeof_MissionValues = sizeof(MissionValues);

static const int StateValuesAddress = sizeof_MissionValues + MissionValuesAddress;
static const int sizeof_StateValues = sizeof(StateValues);

static const int TotalEEPROMStorage = sizeof_Configuration + sizeof_MissionValues + sizeof_StateValues + sizeof_VesselUsageCounters;


void Save_EEPROM_VesselUsage(void)
{
	VesselUsageCounters.PortRudderCounter = PortRudderUsage.Counter ;
	VesselUsageCounters.StarboardRudderCounter = StarboardRudderUsage.Counter;
	VesselUsageCounters.TrimTabCounter = TrimTabUsage.Counter;

	EEPROM_write(VesselUsageCountersAddress, VesselUsageCounters);
};

void Load_EEPROM_VesselUsage(void)
{
	EEPROM_read(VesselUsageCountersAddress, VesselUsageCounters);
	delay(30);

	// check if the init byte has been initialised previously.
	// This is done by using an abitrary value that would not occur randomly e.g. 15. more likely uninitialised values would be either 00,FF,55,AA.
	// I have not verified this on this processor.
	// if not then it implies that it has never been saved.
	if (VesselUsageCounters.init_flag != 15)
	{
		VesselUsageCounters.init_flag = 15;
		VesselUsageCounters.intervalCounter = 0;
		VesselUsageCounters.PortRudderCounter = 0;
		VesselUsageCounters.StarboardRudderCounter = 0;
		VesselUsageCounters.TrimTabCounter = 0;
		VesselUsageCounters.SpareCounter1 = 0;
		VesselUsageCounters.SpareCounter2 = 0;
		VesselUsageCounters.SpareCounter3 = 0;
	};

	PortRudderUsage.Counter = VesselUsageCounters.PortRudderCounter;
	StarboardRudderUsage.Counter = VesselUsageCounters.StarboardRudderCounter;
	TrimTabUsage.Counter = VesselUsageCounters.TrimTabCounter;
}

void Save_EEPROM_ConfigValues(void)
{
	// set the EEEPROM structure storage version number before saving.
	Configuration.EEPROM_Storage_Version = EEPROM_Storage_Version_Const;

	EEPROM_write(ConfigurationAddress, Configuration);
}

void Load_EEPROM_ConfigValues(void)
{
	EEPROM_read(ConfigurationAddress, Configuration);
	delay(30);
}

void load_EEPROM_Mission(void)
{
	// load the mission structure from the EEPROM.
	// perform a simple validation check to ensure that the number of mission steps is within the maximum allowed.
	// V1.0 8/10/2016 John Semmens

	EEPROM_read(MissionValuesAddress, MissionValues);
	delay(30);

	// check if number of mission steps looks valid; otherwise zero it out.
	if (MissionValues.mission_size > MaxMissionCommands)
		MissionValues.mission_size = 0;
}

void Save_EEPROM_Mission(void)
{
	EEPROM_write(MissionValuesAddress, MissionValues);
}

void Load_EEPROM_StateValues(void)
{
	// load the vessel state values. Perform a simple validity check by only setting current mission values if there is a mission.
	EEPROM_read(StateValuesAddress, StateValues);
	delay(30);

	if (MissionValues.mission_size == 0)
	{
		StateValues.mission_index = 0;
		StateValues.StartingMission = false;
		StateValues.CommandState = VesselCommandStateType::vcsFullManual;
	}
}

void Save_EEPROM_StateValues(void)
{
	EEPROM_write(StateValuesAddress, StateValues);
}

bool EEPROM_Storage_Version_Valid(void)
{
	// return true or false to indicate if the cureent stored data structures have a version consistent with the current software
	// V1.0 28/8/2016 John Semmens.
	return (EEPROM_Storage_Version_Const == Configuration.EEPROM_Storage_Version);
}

void Load_Config_default_values(void)
{
	// load default Config values when the dtored values are invalid.
	// V1.0 28/9/2016 John Semmens.
	// V1.1 11/10/2016 added MaxFileSize.
	// V1.2 29/10/2017 remove items related to MPU-2950
	// V1.3 30/10/2017 added compass offset angle
	// V1.4 6/12/2017 added Motor speed mapping 
	// V1.5 17/1/2018 added compass calibration defaults
	// V1.6 18/2/2017 updated default values for motor speed
	// V1.7 27/4/2018 updated TargetHeadingFilterConstant following on-water trials.
	// V1.8 28/10/2018 added timezone_offset.
	// V1.9 21/2/2019 added TackingMethod

	Configuration.SDCardLoggingMask = 49135; // enable all SD Card logging
	Configuration.TelemetryLoggingMask = 3215; // Enable enough for the Dashboard
	Configuration.TackingMethod = ManoeuvreType::mtGybe;
	Configuration.MinimumAngleDownWind = 20; // degrees off dead downwind
	Configuration.MinimumAngleUpWind = 40; // degrees off head to wind.
	Configuration.WPCourseHoldRadius = 10; // metres radius from WP - hold course
	Configuration.MinimumWindChangeTime = 60;// seconds
	Configuration.RTHTimeManualControl = 180; // seconds  - 3 min
	Configuration.DefaultMaxCTE = 20; // 20 metres. used for Return to Home for example
	Configuration.SaveStateValues = true;
	Configuration.CompassOffsetAngle = 0; // degrees
	Configuration.WindAngleCalibrationOffset = -90; // degrees
	Configuration.DisplayScreenView = 'm'; // mission overview/ checklist
	Configuration.SDCardLogDelimiter = '\t';  //tab --  perhaps  ',' or '\t'
	Configuration.TargetHeadingFilterConstant = 0.02;

	Configuration.SailableAngleMargin = 25; // was 45°, changed to 35° 15/4/2020 , changed to 25° 5/1/2020

	Configuration.timezone_offset = 10; // offset to our timezone from UTC/GPS time +10 hours for EST, and +11 for EDT (summer time).

	Configuration.MagnetVariation = 12; // about 12 degrees East for Port Philip

	Configuration.MaxFileSize = 2048; //  was 1024 kb 

	Configuration.RC_FailSafe_Ch0 = 1600; //  micro seconds -- rudder
	Configuration.RC_FailSafe_Ch1 = 1500; //  micro seconds
	Configuration.RC_FailSafe_Ch2 = 1500; //  micro seconds
	Configuration.RC_FailSafe_Ch3 = 1450; //  micro seconds - drive motor controller
	Configuration.RC_FailSafe_Time = 10; //  seconds

	Configuration.RC_IN_Channel_Command = 0;	 // channel number
	Configuration.RC_IN_Channel_Steering = 1;	 // channel number
	Configuration.RC_IN_Channel_Sail = 2;		 // channel number
	Configuration.RC_IN_Channel_Motor = 3;		// channel number

	Configuration.Servo_Channel_Steering = 0;		// channel number
	Configuration.Servo_Channel_Steering_Stbd = 1;	// channel number
	Configuration.Servo_Channel_Sail = 2;			// channel number
	Configuration.Servo_Channel_Motor = 3;			// channel number

	Configuration.RC_IN_Command_Threhold0 = 900;  // microseconds
	Configuration.RC_IN_Command_Threhold1 = 1100; // microseconds
	Configuration.RC_IN_Command_Threhold2 = 1300; // microseconds
	Configuration.RC_IN_Command_Threhold3 = 1700; // microseconds
	Configuration.RC_IN_Command_Threhold4 = 1950; // microseconds
	Configuration.RC_IN_Command_Threhold9 = 2100; // microseconds

	Configuration.DualRudder = false; // Single Rudder only

	Configuration.CommandPort = 0;  // serial port 0
	Configuration.BluetoothPort = 2; // Serial Port2
	Configuration.SatCommsPort = 3; // serial port 3 // radio telemetry port

	Configuration.CommandPortBaudRate = 9600; // Baud Rate .e.g.  57600 Baud, 9600 Baud
	Configuration.BTPortBaudRate = 9600; // Baud Rate
	Configuration.SatCommsPortBaudRate = 9600; // changed from 57600Bd for the std 433MHz telemetry radios to 9600Bd for LoRa;  // Baud Rate

	// Steering PID
	//adjust PID 25/4/2018 8,0,0
	Configuration.pidKp = 8; // Proportional //adjust PID 25/4/2018
	Configuration.pidKi = 0; // Integral
	Configuration.pidKd = 0; // Differential
	Configuration.pidOutputmin = -400;  // steering PID output limits - microseconds - min was 1100us
	Configuration.pidOutputmax = +400;  // steering PID output limits - microseconds - max was 1900us
	Configuration.pidDirection = REVERSE; // Direction: DIRECT 0 or REVERSE 1
	Configuration.pidCentre = 1500; // 

	Configuration.FailsafeCommandState = VesselCommandStateType::vcsFollowMission; // mission state on RC Failure

	Configuration.UseMotor = false; // set default to true during early trials

	// load some reasonable calibration values for the compass
	Configuration.mx_min = -6400;
	Configuration.mx_max = 6400;
	Configuration.my_min = -10000;
	Configuration.my_max = 10000;
	Configuration.mz_min = 0;
	Configuration.mz_max = 0;

	Configuration.LoiterRadius = 15; // metres

	Configuration.TWD_Offset = 30; // degrees. about 30 degrees

	// Scale and Offset for mapping trim tab angle to the Servo input signal in microseconds
	Configuration.TrimTabScale = 20; // us/degree.  
	Configuration.TrimTabOffset = 1500; // us
	Configuration.TrimTabDefaultAngle = 15; // degrees

	Configuration.CTE_CorrectionGain = 20; // °  degrees. apply 20° correction to the CTS when CTE/CTEmax = 1
};

void Load_ConfigValues(int CommandPort, bool SerialLog)
{
	// Load the Config values from EEPROM
	// perform a simple check on the validity of the stored data by using a stored structure version number.
	// V1.0 8/10/2016 John Semmens
	// V1.1 23/10/2016 Updated to provide option to display messages.

	Load_EEPROM_ConfigValues();
	if (SerialLog)
	{
		(*Serials[CommandPort]).print(F("MSG,Current EEPROM Structure Version: "));
		(*Serials[CommandPort]).print(Configuration.EEPROM_Storage_Version);
		(*Serials[CommandPort]).print(F(". Expected: "));
		(*Serials[CommandPort]).print(EEPROM_Storage_Version_Const);
		(*Serials[CommandPort]).print(F(". "));
		(*Serials[CommandPort]).println((EEPROM_Storage_Version_Valid() ? "OK." : "Loading Default Values."));
	}
	// load other structures from the EEPROM provided the structure storage version.
	if (EEPROM_Storage_Version_Valid())
	{
		if (SerialLog)
		{
			(*Serials[CommandPort]).print(F("MSG,Loading Mission.."));
		}

		load_EEPROM_Mission();

		if (SerialLog)
		{
			(*Serials[CommandPort]).print(MissionValues.mission_size);
			(*Serials[CommandPort]).println(F(" steps."));
		}
		// send the mission command list to the serials device.
		CLI_Processor(Configuration.CommandPort,"mcl,");

		if (Configuration.SaveStateValues)
		{
			if (SerialLog)
			{
				(*Serials[CommandPort]).println(F("MSG,Loading State Values."));
			}
			Load_EEPROM_StateValues();
		}
	}
	else
	{
		// the stored configuration is invalid, so load some reasonable default values
		Load_Config_default_values();
	}
};

