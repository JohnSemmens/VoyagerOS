// configValues.h

#ifndef _CONFIGVALUES_h
#define _CONFIGVALUES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Navigation.h"
#include "CommandState_Processor.h"

static const int EEPROM_Storage_Version_Const = 61; // change this number to force config to be cleared and revert to default.

struct configValuesType {
	byte EEPROM_Storage_Version = EEPROM_Storage_Version_Const; // stored object version. this is to test if the data being retrieve is valid with reference to this version. 

	char DisplayScreenView; // character code representing the current Local Display Screen
	word TelemetryLoggingMask; // mask value for enabling logging of different data items via the telemetry link
	word SDCardLoggingMask; // mask value for enabling logging of data item to the SD Card.

	char SDCardLogDelimiter; // Delimiter character for the SD Card Log files.

	bool SaveStateValues;  // set whether the current state of the vessel should be saved. set to False for testing. Set to True for real mission.

	byte timezone_offset; // offset to our timezone from UTC/GPS time +10 hours for Eastern Standard Time, and +11 for Eastern Dalylight Saving Time.

	// sailing limits
	ManoeuvreType TackingMethod; // this describes the method tacking. i.e. tack or gybe
	byte MinimumAngleUpWind;	// minumum angle off the wind when close hauled. Typically about 45 degrees. Zero means its posible to sail directly upwind.
	byte MinimumAngleDownWind;	// minimum angle off dead down wind. Force the vessel to tack down wind. Zero means no constraint.
	int WPCourseHoldRadius;		// metres - Waypoint Course Hold Radius - don't change course inside this circle.

	int RTHTimeManualControl;  // Time under Manual Control (Steer Wind or Steer Compass) to Return To Home (RTH) 
								// This is a filesafe to cause the vessel to Return To Home if there is no further communications (i.e. loss of signal)

	int DefaultMaxCTE; // Max CTE for cases where it is not explicitly set. E.g. Return to Home

	int MinimumWindChangeTime;		 // sec - min time to elaspse before changing sailing state due to an apparent wind change.

	int TWD_Offset;   // degrees -  offset from AWA to TWD. about 30 degrees. This is a hack to guess TWD. 

	float TargetHeadingFilterConstant; // Filter constant the Target Heading for the Helm versus the CTS value.

	byte SailableAngleMargin; //degrees. this to inhibit tacking when the angle is marginal. i.e. ensure angle is not marginal.

	// Calibration values
	int WindAngleCalibrationOffset; // Calibration Offset for the Wind Angle Indicator

	// Compass rotation offset configuration
	int CompassOffsetAngle; // Compass rotation offset configuration e.g. 0 or 180 degrees

	float MagnetVariation;			// around 12 degrees for Port Philip. Positive values are East. 

	int MaxFileSize; // Maximum file size for log files on the SD Card. // kbytes 1024 bytes.

	// RC Fail safe values for each channel. Microseconds typically 1000 to 2000 microseconds. 
	// Use a value of zero for no setting.
	uint16_t RC_FailSafe_Ch0; 
	uint16_t RC_FailSafe_Ch1;
	uint16_t RC_FailSafe_Ch2;
	uint16_t RC_FailSafe_Ch3;

	uint16_t RC_FailSafe_Time; // time before failsafe setting are actioned. Seconds. typically 10s.

	int RC_IN_Channel_Command; // channel for RC Command; 0
	int RC_IN_Channel_Steering; // steering input channel  1
	int RC_IN_Channel_Sail; // sail control channel 2
	int RC_IN_Channel_Motor; // Motor control channel 3

	int Servo_Channel_Steering; // channel steering and port steering channel in the case of dual rudder servos is true 
	int Servo_Channel_Steering_Stbd; //  starboard sterering channel in the case of dual rudder servos is true 
	int Servo_Channel_Sail; // sail control channel 2
	int Servo_Channel_Motor; // Motor control channel 3

	uint16_t RC_IN_Command_Threhold0; //  900us
	uint16_t RC_IN_Command_Threhold1; // 1100us
	uint16_t RC_IN_Command_Threhold2; // 1300us
	uint16_t RC_IN_Command_Threhold3; // 1700us
	uint16_t RC_IN_Command_Threhold4; // 1950us
	uint16_t RC_IN_Command_Threhold9; // 2100us

	//// Motor Electronic Speed Controller - Speed mapping - approximate percentages to microseconds from servo output
	//// this is used during mission command execution to set the motor speed
	//uint16_t Motor_Ahead_100;	// 2000 us
	//uint16_t Motor_Ahead_75;	// 1825 us
	//uint16_t Motor_Ahead_50;	// 1750 us
	//uint16_t Motor_Ahead_25;	// 1625 us
	//uint16_t Motor_Stop;		// 1500 us
	//uint16_t Motor_Reverse_25;	// 1375 us

	VesselCommandStateType FailsafeCommandState; // default mission state on RC Failure

	bool DualRudder; // False: means normal single rudder system. True: means Dual Rudder system with auto cut-off.
	bool UseMotor;	 // True: means that a drive motor has been installed and is available to use.

	int CommandPort; // Serial Port for CLI - verbose
	int BluetoothPort; // Serial Port for Bluetooth Module
	int SatCommsPort; // Serial Port for Satellite Communications - terse 

	uint32_t CommandPortBaudRate;	// Baud Rate .e.g. 57600 Baud,  9600 Baud
	uint32_t BTPortBaudRate;		// Baud Rate
	uint32_t SatCommsPortBaudRate;  // Baud Rate

	// Steering PID
	double pidKp, pidKi, pidKd;			// steering PID Constants
	double pidOutputmin, pidOutputmax;   // steering PID output limits
	int pidDirection;
	double pidCentre; // set this to be the steering servo neutral position

	long LoiterRadius; // maximum loiter radius when using the "Loiter here" Vessel Command (not the Mission Command)

	// Voltage Measurements 
//	double VmScale0, VmScale1; // Voltage measurement scale factors
//	int VmADCPin0, VmADCPin1; // ADC Pin numbers e.g. A10 and A11 are represented as 10 and 11.

//	double VmScalePowerSensorV, VmScalePowerSensorI;  // Voltage measurement scale factors
//	int PowerSensorVPin, PowerSensorIPin; // ADC Pin numbers for Power Sensor Current and Voltage pins - V=8 I=9
	
	int TrimTabScale; // Scale and Offset for mapping trim tab angle to the Servo input signal in microseconds
	int	TrimTabOffset; 

	int TrimTabDefaultAngle;

	float mx_min;	// Magnetic Compass Calibration limits. mpu 9250 WingSail Angle
	float mx_max;
	float my_min;
	float my_max;
	float mz_min;
	float mz_max;	// 

	int CTE_CorrectionGain; // this the gain of the CTE steering correction adjustment
};

/* Storage Map for EEPROM
	
	1. VesselUsageCounters				(Address 0)    size  32
	2. Configuration Values Structure   (Address 32)   Size 288
	3. Mission Array				    (address 320)  Size 968
	4. Vessel State Values Structure    (address 1288) Size  36
			Total:	   							           1324 bytes
*/


void Save_EEPROM_VesselUsage(void);
void Load_EEPROM_VesselUsage(void);

void Load_EEPROM_ConfigValues(void);
void Save_EEPROM_ConfigValues(void);

void load_EEPROM_Mission(void);
void Save_EEPROM_Mission(void);

void Load_EEPROM_StateValues(void);
void Save_EEPROM_StateValues(void);

// return true or false to indicate if the current stored data structures have a version consistent with the current software
bool EEPROM_Storage_Version_Valid(void);

void Load_Config_default_values(void);
void Load_ConfigValues(int CommandPort,bool SerialLog);


#endif

