// Voyager Operating System
// Operating System for Sailing vessels.

// V0.1.3 15/7/2016 
// V0.1.4 24/7/2016	
// V0.1.5 30/7/2016
// V0.1.6 31/7/2016
// V0.1.15 24/8/2016 Changed from ATMega328 to ATMega2560 because of memory requirements.
// V0.1.16 28/8/2016 
// V0.1.17 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V0.1.18 2/9/2016 continued development
// V0.1.19 12/9/2016 added support for SD Card logging
// V0.1.20 8/10/2016 
// V0.1.21 18/10/2016 added RC IN Channels and Servo Out Channels. commenced adding command state handler.
// V0.1.22 22/10/2016 setup Serial Ports as an index array to allow redirection of serial streams by configuration.
// V0.1.23 23/10/2016 continued rollout of serial array
// V0.1.24 31/10/2016 Setup True Heading and update logging details
// V0.1.25 2/11/2016 added SD Card Presence Flag to inhibit using the Card if not present.
// V0.1.26 11/11/2016 ironing out bugs in the mission processing.
// V0.1.27 13/11/2016 Setup steering and PID - no tacking algorithm yet
// V1.0.28 16/11/2016 First Operational Release - No reference to wind angles. No tacking.
// V1.0.29 22/11/2016 Minor updates for improved handling of missing hardware.
// V1.0.30 12/1/2017 Added Init Routine for Magnetic Angle sensor to indicate if its operational
//					Improved Mission Index Handling
// V1.0.31 28/10/2017 change the IMU from MPU9250 to Adafruit BNO-055.
// V1.0.32 30/10/2017 added Compass rotation offset configuration
// V1.0.33 8/11/2017 added Compass calibration and saving a loading of calibration values
// V1.0.34 12/11/2017 updated LCD Nav Display. Updated to change BRL to RLB. 
//					Added Home Loc LCD Page. Added hlc command to clear home location
//					Removed Mission Command for Set Home Location - it didn't make sense.
//					Added command mcp for mission plotting.
// V1.0.35 5/12/2017 Adding support for Motor Drive - assign pins for RC. Setup enumerated type. set up mapping for ESC.
//					Add motor speed to mission command execution.
// V1.0.36 13/12/2017 adding support for OLED display 
// V1.0.37 17/12/2017 continued rollout of drive motor control
// V1.0.38 11/1/2018 Commencing to add Loitering.
// V1.0.39 13/1/2018 added support for altering drive motor cruise speed setting.
// V1.0.40 22/1/2018 added new lcd/oled command lcd,d
// V1.0.41 24/1/2018 continued rollout of Loitering with the motor
// V1.0.42 24/1/2018 completed first iteration of a functioning system with Loiter with Motor
// V1.0.43 24/1/2018 extending loitering from vessel command to mission commands
// V1.0.44 2/2/2018 fixed update waypoint bug
// V1.0.45 18/2/2018 updated drive motor speed default values
// V1.0.46 2/4/2018 Added support for ADC for voltage and current measurements
// V1.0.47 2/4/2018 Added support for ADC for voltage and current measurements for the APM Power Sensor
// V1.0.48 4/4/2018 Added Telemetry Logging for Servo Outputs.
// V1.0.49 9/4/2018 Added Telemetry Logging for Voltage Measures Outputs.
// V1.0.50 14/4/2018 bug fix in SYS telemetry
//					 bug fix in parameter get/set for Voltage measurement scaling factor - data type is now floating point.		
//					 tweak default values for Voltage measurement scaling factors
// V1.0.51 15/4/2018 bug fix in SD Card logging for VLT
//					 tweak default values for steering limits
// V1.0.52 18/4/2018 bug fix in SD Card logging for VLT and SVO
// V1.0.53 24/4/2018 work on PID after first trial, added steering Neutral (pidCentre) parameter, fixed telemetry bug in Decision events
// V1.0.54 28/4/2018 refactored design of Mission step handling. and many minor bug fixes while running simulations
// V1.1.55 25/5/2018 Update Telemetry SIT message to include AWA
// V1.1.56 26/5/2018 Added Steer Wind Course Command 
// V1.2.57 2/6/2018 added code to setup waypoints and resume a mission after a reboot.
// V1.3.58 21/9/2018 bug fix for mission loiter command not stopping motor on outbound
// V1.3.59 19/10/2018 fix problem where a missing telemetry Transceiver would prevent boot up.
// V1.4.60 20/10/2018 Change IMU from BNO055 to BNO080.
// V1.4.61 20/10/2018 clean up unused files.
// V1.5.62 21/10/2018 change GPS from Serial to I2C
// V1.5.63 21/10/2018 added MaxCTE to WAY Telemetry message.
//                                 changed test for "past waypoint" to include a test for being within range of the waypoint.
// V1.5.63a 21/10/2018 bug - fix add setting of MaxCTE
// V1.6.64 21/10/2018 skipped
// V1.6.65 21/10/2018 Remove the RTC and use the GPS for provision of Time.
// V1.7.67 22/10/2018 fixed compile errors 
// V1.7.68 23/10/2018 changing over from floatToString.h to dtostrf()
// V1.7.69 24/10/2018 completed change over from floatToString.h to dtostrf()
// V1.7.70 28/10/2018 changed over from GPS time to local time. Commenced adding bluetooth support.
// V1.8.71 29/10/2018 update the next WP for display purposes when not yet following a mission, but when setting mission index
// V1.9.72 10/11/2018 adding Bluetooth support.
// V1.9.73 10/11/2018 adding the MPU9250 as a Magnetic Angle Sensor for the WingSail. it uses I2C and runs on 3V3.
//						Removed the AS5030 Magnetic Angle Sensor because it requires 5V.
// V1.9.74 25/11/2018 integration testing of the wingsail, sensors and actuators.
// V1.9.75 26/11/2018 further integration testing of the wingsail
// V1.9.76 27/11/2018 finalising first version of Steering with constant AWA.
// V1.9.77 28/11/2018 commence adding two-way comms with Wingsail.
// V1.9.78 3/12/2018 changed strcpy to strncpy within the CLI processes to guard against corrupting memory with long strings
//					 Increase length of Command line parameters to 12.
// V1.10.79 22/12/2018 Changed Voltage/Current Measurement from ADC to I2C INA219
// V1.11.80 10/1/2019 disable the Drive Motor by default, and ensure that it only operates in manual mode, to allow Sail as primary  
// V1.12.81 18/1/2019 minor updates to OLED display pages
// V1.12.82 18/1/2019 debug code in 9250 imu init
// V1.12.83 20/1/2019 added MagAccuracy to the IMU BNO080 and Attitude Display -- doesn't seem useful though.
// V1.13.84 5/2/2019 start removing motor software. 
//					 work on improving wingsail performance during tacking manoeuvre. 
// V1.13.85 8/2/2019 further work on improving wingsail performance during tacking manoeuvre. 
// V1.14.86 10/2/2019 start adding sailing/tacking parameter measurements
// V1.14.87 10/2/2019 added tacking concepts into CalculateSailingCTS
// V1.14.88 10/2/2019 updates to tacking following code review 
// V1.14.89 11/2/2019 tidy up prior to sea trials.
// V1.14.90 12/2/2019 fix logic reversal in tacking decisions.
// V1.14.91 13/2/2019 break out tacking code into a new procedure.
// V1.15.92 13/2/2019 Commence adding Sailing Loitering 
// V1.16.93 14/2/2019 Complete Loitering and add Gybe control.
// V1.16.94 15/2/2019 Bug Fix - Turn Heading not being passed through.
// V1.17.95 15/2/2019 Add Control Mask into Mission structure for future use in power control etc.
// V1.18.96 15/2/2019 add Wear Logging for tracking Servo usage.
// V1.18.97 19/2/2019 write usage data to EEPROM, and add usage logging to SD and Tel logging
// V1.18.98 17/3/2019 add support for MPU-9255 for wind angle sensor in addition to MPU-9250.
// V1.19.99 24/3/2019 support new PCB V1.0, add power management
// V1.20.100 6/4/2019 Continued set up on new PCB.
// V1.20.101 6/4/2019 Fixed initialisation lockup problem. added delay to GPS Init.
// V1.20.102 7/4/2019 Updated Loiter Here Behaviour
//			set RC off state to loiter here, for testing in: CommandState_Processor.cpp
// V1.20.103 8/4/2019 minor updates to SD logging and loiter radius.
// V1.20.104 15/4/2019 refining TWD calculation to use vectors
// V1.21.105 16/4/2019 Updates to Loitering
// V1.21.106 16/4/2019 Loiter Testing
// V1.21.107 22/04/2019 Tack Testing
// V1.21.108 22/04/2019 developed a new method of handling a tack versus gybe, by tracking the previous CTS vs CTS.
// V1.21.109 23/04/2019 Corrections to SD Card logging.
// V1.21.110 23/04/2019 Added protection to Loitering to inhibit sailing above a sailable course
// V1.21.111 24/4/2019 Added event logging to Loitering
// V1.21.112 25/4/2019 minor correction to OLED Attitude display - A.
// V1.22.113 25/4/2019 change over IMU from BNO080 to the USFS ST Version.*****
// V1.22.114 25/4/2019 added CLI command ccs for Compass Cal Save
// V1.22.115 25/4/2019 minor tidy up - ensure USFS init message are sent to the designated comms serial port.
// V1.22.116 27/4/2019 Version with Temporary I2C GPS - awaiting replacement GPS Titan
// V1.22.117 27/4/2019 Minor fixed after sailing trial. set up VesselCommand State RC Fail Safe as a config item.
// V1.22.118 28/4/2019 Fix Logging error with ENV in SD Card.
// V1.22.119 30/4/2019 improvements to event logging.
// V1.22.120 5/5/2019 added more logging to the SD Card - including changes to parameters. and changes to CommandState. i.e. manual override
// V1.23.121 5/5/2019 Update closehauled sailing to use AWA more directly. Added Trim Tab default Angle as a Config parameter.
// V1.23.122 6/5/2019 updated TWD calculation, and added AWA to TWD offset as a parameter.
// V1.24.123 16/5/2019 added verification of the AWA as part of completing a Gybe.
// V1.24.124 26/5/2019 bug fix oled display layout
// V1.24.125 28/5/2019 added intialise Prev_WP on first GPS Acquisition. Corrected OLED display for TurnHdg.
// V1.24.126 4/6/2019 checkin
// V1.25.127 11/6/2019 Add Trim Tab Angle and Wing Angle to SD Card log.
//					   Updated to prevent course changes while in the waypoint boundary circle
//					   Change tests for boundary to ensure that they are not sensitive to CTE sign. 
//					   Added CLI Command to retreive flags for SD Card and wing angle sensor.
//					   Added Decision Event logging for Holding course within the waypoint boundary circle
//					   Added TackingOffsetAngle to the MinimumAngleUpWind to inhibit tacking a bit. If this concept works then change to a configurable tacking angle
// V1.26.128 15/6/2019 Corrected sign reversal error for calculating port and starboard tacks in the SteerCloseHauled function
// V1.26.129 15/6/2019 Updated the "hold course near WP" bahaviour to ensure its a sailable course to hold.
// V1.27.130 17/6/2019 Updated to reinstate tests for boundary to ensure that they ARE sensitive to CTE sign. 
// V1.27.131 29/6/2019 Minor updates for compass calibration in the USFS device. Added CLI commamds CCS and CCG. Added Compass Accuracy into ATT for Telemetry and SD Card
// V1.28.132 3/7/2019 Added Power Control to the Mission Increments
// V1.28.133 9/7/2019 added Decision Event Reason value for logging.
// V1.29.134 11/7/2019 Added Roll Angle dampened value and SD Card logging for it.
//					   Added Heading Error based on GPS data, and apply to Heading as a correction
// V1.29.135 12/7/2019 Updated SD Card logging to include HDG_Mag and HDG_Err.
// V1.30.136 16/7/2019 adding the new Mission command for Steering a Wind Angle
// V1.30.137 19/7/2019 Add parameter for DTW Course Hold Radius, and changed DTW Course Hold Radius to use parameter and not boundary
//					  Updated startup to delay if equipment fault found
// V1.31.138 29/7/2019 Added Compass Detail screen 'c' to OLED
// V1.31.139 1/8/2019 ensure that the RC Rx and the Telemetry is switched on at end of mission.
// V1.31.140 2/8/2019 added OLED dispaly to show current Mission Command lcd,o
// V1.31.141 2/8/2019 added MIS sentence to SD Card Logging
// V1.31.142 6/8/2019 added start up logging to the SD Card of OS version and currrent usage.
// V1.31.143 10/11/2019 moved to the version and usage logging from startup into start of each log file. Changed default log size to 2Mb.
// V1.31.144 16/11/2019 update SD Crd Log MIC record to use sting value for Mission command
// V1.31.145 8/12/2019 added Timezone as a controllable parameter #7
// V1.31.146 8/12/2019 updated "m" LCD commnd to include mission size.
// V1.31.147 8/12/2019 Add Save State when setting mission index via CLI
// V1.31.148 15/12/2019 update to allow the mission index to be overriden during a mission, to skip a waypoint, revert to a waypoint.
//						Added "Sailable Angle Margin" as a config item and set default to 45 degrees, rather than the hardcoded 20 degrees
// V1.31.149 17/12/2019 check in.
// V1.31.150 19/12/2019 adjust default config values. 
//			 14/3/2020 Voyager Sailed across Port Phillip for 40 hours, 30 miles. Several design changes were identified as a result.
// V1.32.151 10/4/2020 update for improve CTE calculation an resolution.
// V1.32.152 11/4/2020 adding in CTE Correction to improve course keeping, particulary with long distances to waypoints.
// V1.32.153 15/4/2020 reduce SailableAngleMargin default value from 45 to 35 degrees.
//						altered the IsBTWSailable to reduce SailableAngleMargin when CTE exceeds the CTE max boundary.
// V1.33.154 19/5/2020 Changed telemetry to be pull rather than push (or broadcast).
// V1.33.155 8/6/2020 changed Radio telemetry Baud rate default to 9600 to suit default LoRa from 57600.
// V1.33.156 8/6/2020 add delays to list data being returned to improve behaviour with LoRa. WARNING *** 
//						*** WARNING ***  this is not cooperative and will stop normal porcessing for extended periods of time.
//						Considering creating an outgoing message queue to allow for asynchronous operation with delays.
// V1.33.157 11/6/2020 First version to implement the outgoing message queue.
// V1.34.158 14/6/2020 Ready for trialing with LoRa Telemetry Radio
// V1.35.159 14/6/2020 Change from SD.h to SDFat.h, because the SD library appears to have been superseded by SDFat Library.
// V1.35.160 25/6/2020 changed dead to wind region from 5 degrees to 3 degrees
// V1.36.161 24/10/2020 added AWD, Apparent Wind Direction, relative to north. This is for use while gybing, to improve gybe performance.
// V1.36.162 22/11/2020 update default time zone for Daylight Savings
// V1.36.163 27/11/2020 bug fix: add missing response for parameter 59.
// V1.37.164 2/1/2021 Commence adding downwind tacking
// V1.37.165 5/1/2021 updated to support Downwind tacking, hence four laylines.
// V1.37.166 8/1/2021 minor tweaks
// V1.37.167 10/1/2021 Change behaviour of the flag to use simulated GPS, to continue reading GPS but save the time only. This is to allows the time stamp on logs to continue.
// V1.37.168 10/1/2021 removed decision envents from th Telemetry logging because they weren't using the message queue and may have been responsible for corruption of the telemetry module.
//						This didn't help. There was no corruption of message queues. The telemetry was being powered off in the mission - doh! One day lost.
// V1.37.169 10/1/2021 new product version for testing upwind/downwind tacking
// V1.37.170 10/1/2021 new version for sea trials
// V1.37.171 11/1/2021 add OLED screen 2, to help verify TWD calculation.
// V1.37.172 11/1/2021 bug fix for reversal while tacking downwind.
// V1.37.173 13/1/2021 Update startup process to reset initial location if we're still on mission step zero, while following mission.
// V1.37.174 17/1/2021 added logging for Changes to Compass Course Steering or Wind Steering.
// V1.37.175 18/1/2021 updated to force choice of steering favoured tack after incrementing a mission step. 
//						This problem was causing it sail well past a waypoint before adopting a good course.
// V1.37.176 18/1/2021 add new LoRa command LPF, to return Sail Performance data to assist with tuning and polar diagram preparation.
// V1.37.177 27/1/2021 add support for setting trimtab default value within the SCS Steer Wind course command
// V1.37.178 28/1/2021 correction to LPF message response.
// V1.38.179 1/2/2021 bug fix - SCS command was causing trim tab default to be changed (or zeroed out) on all commands, not just Steer Wind.
// V1.38.180 7/2/2021 Add new Config item RTHTimeManualControl as failsafe timeout while operating Steer Wind or Steer Compass.
//						Added reset of Misson Command timer in CLI, when a Vessel Command State message is received.
//						Added change to Return to Home when time out occurs on Steer Compass or Steer Wind.
// V1.39.181 2/4/2021   Added Bearing and Distance to Home to the Navigation stucture, if Home is set.
//						These are included in the telemetry HLG message.
// V1.39.182 5/4/2021 Added Config Item DefaultMaxCTE for cases where MaxCTE is not explicitly set, such as Return to Home.
// V1.40.183 7/4/2021 Commence adding simulation components for weather and vessel.
// V1.40.183 12/4/2021 First working version of the integrated simulator. Check in.
// V1.40.184 12/4/2021 Release version installed on vessel.
// V1.40.185 16/4/2021 Corect SD card logging - Steering to Wind to Compass timeout

// Build Notes: use Visual Studio 2019

// I2C DEVICE MAP:
//		0x10 GPS Titan // currently not in use, it was interferring with other I2C devices on 0x28
//		0x28 EM7180 USFS
//		0x3C OLED Display
//		0x40 INA219 Current/Voltage Sensor
//		0x68 MPU9250/MPU9255 Wing Angle sensor
//		0x29 Temporary GPS

char Version[] = "Voyager OS 1.40.185";
char VersionDate[] = "12/4/2021";

#include "Loiter.h"
#include "Local_Display.h"
#include "OLED_Logging.h"
#include "Steering.h"
#include "CommandState_Processor.h"
#include <Servo.h>
#include "RadioControl.h"
#include "LEDHeartBeat.h"
#include "SailingNavigation.h"
#include "location.h"
#include "Waypoints.h"
#include "Navigation.h"
#include "USFS_IMU.h"
#include "TelemetryLogging.h"
#include "configValues.h"
#include "SchedulerCooperative.h"
#include "CLI.h"
#include "GPS.h"
#include "GPS_Temp.h" // temporary GPS
#include <Wire.h>
#include "configValues.h"
#include <EEPROM.h>
#include "eepromAnything.h"
#include "Mission.h"
#include <SPI.h>
#include "SparkFun_I2C_GPS_Arduino_Library.h" 
#include <SdFat.h>
#include "SDCardLogFile.h"
#include "SailingNavigation.h"
#include "Adafruit_GFX.h"				// added for OLED display
#include "Adafruit_SSD1306.h"			// added for OLED display
#include "TinyGPS++.h"
#include "Wingsail.h"
#include "BluetoothConnection.h"
#include "MPU9250.h"
#include "WindAngle_2950.h"
#include "Adafruit_INA219.h"
#include "VI_Measurement.h"
#include "WearTracking.h"
#include "PowerManagement.h"
#include "TelemetryMessages.h"
#include "sim_vessel.h"
#include "sim_weather.h"

TinyGPSPlus gps;						// The TinyGPS++ object
I2CGPS myI2CGPS;
bool UseSimulatedVessel = false;			// flag to disable the GPS and indicate that the current location is simulated 
										// but keep reading the GPS to get current time.

sim_vessel simulated_vessel;		// Simulated vessel object
sim_weather simulated_weather;		// Simulated weather object

Time CurrentUTCTime;					// Current Time from GPS. We'll call it UTC, even though that's not strictly the same.
Time CurrentLocalTime;				// Current Local Time converted from GPS - UTC/GPS time

configValuesType Configuration;		// stucture holding Configuration values; preset variables
MissionValuesStruct MissionValues;	// structure holding mission details
StateValuesStruct StateValues;		// structure holding Vessel state information. This is used to recover from a restart part way through a mission.

DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
int DecisionEventValue;							// this a value relevant to an event
int DecisionEventValue2;						// this a second value  relevant to an event

// USFS IMU - Define function pointers for class instances
I2Cdev      *i2c_0;
EM7180      *Sentral_0;
IMU         *imu_0;
IMUStruct	myIMU;

NavigationDataType NavData; // stucture containing the current nav data, including current loc, next and prev waypoints etc.

SdFat sd;
SdFile LogFile;
RC_IN_Type RC_IN;
bool SD_Card_Present; // Flag for SD Card Presence

Adafruit_INA219 ina219;
double PowerSensorV=0, PowerSensorI=0; // APM Power Sensor Voltage and Current Measurement results.

// Loop Timer Contants used by the scheduler
static const int SlowLoopTime = 5000;  //ms 5 seconds
static const int MediumLoopTime = 1000; //ms 1 second
static const int FastLoopTime = 25;  //ms  50 ms
static const int LoggingLoopTime = 1000; //ms 1 second
static const unsigned long SlowLoggingLoopTime = 600000; //ms 10 minutes 
static const unsigned long SlowLoggingStartDelay = 30000; // ms 30 seconds start up delay
static const int TelemetryLoopLoopTime = 750;  //ms 1 seconds


long loop_period_us; // microseconds between successive main loop executions

HardwareSerial *Serials[4]; // array for serial ports
int CommandPort;    // Command port serial port. These are separate variables from the config values.
uint32_t CommandPortBaudRate; // Command port serial baud rate.

// strings used to hold the display messages for the LCD/OLED, required the command "dsp"
char MessageDisplayLine1[10];
char MessageDisplayLine2[10];

LoiterStruct LoiterData;

double SteeringServoOutput;

byte BluetoothStatePin = 47; // input_pullup
BTStateType BTState = Idle;

MPU9250 WingSailAngleSensor;		// The IMU including Magnetic Compass and Gyro 
WingSailType WingSail;

byte OLED_PowerPin = 38; // D38
byte RC_PowerPin = 23; // D23
byte Telemetry_PowerPin = 24; // D24
byte GPS_PowerPin = 40; // D40
byte LEDpin = 13;

WearCounter PortRudderUsage;
WearCounter StarboardRudderUsage;
WearCounter TrimTabUsage;
VesselUsageCountersStruct VesselUsageCounters;

byte MessageArray[EndMarker];


void SlowLoop(void *) // 5 seconds
{	
	// check if the mission has advanced to the next step
	// and update the next and previous waypoints needed for navigation.
	MissionUpdate();

	// Update the Navigation Object data
	// update the Range, Bearing and Cross Track Error to next waypoint to establish required heading 
	// and check if the current mission command has been completed, and advance the mission index if required.
	NavigationUpdate_SlowData();

	// calculate the best course to steer, including all tacking decisions this sets NavData.CTS
	UpdateCourseToSteer();

	BluetoothManageConnection(Configuration.BluetoothPort);

	// Update Sail Settings e.g set the sails in accordance with the current point of sail.
	wingsail_update();
}

void MediumLoop(void *)  // 1 second
{
	UpdateLocalTimeFromGPSTime(Configuration.timezone_offset);

	// increment the tack timer
	NavData.TackDuration = NavData.TackDuration + 1;

	// process serial command messages
	CLI_Process_Message(CommandPort);

	// Process Vessel Command State Changes from either RC or Telemetry Radio Command Line Interpreter
	CommandState_Processor();

	// Get the AWA and TWD
	GetApparentWind();
	GetTrueWind();

	NavigationUpdate_MediumData();

	// 1 second medium loop - perform the wingsail_update if we are in a Manual Mode (where the vessel is controlled manually)
	// and needs to be more responsive.
	// Update Sail Settings e.g set the sails in accordance with the current point of sail.
	if ((StateValues.CommandState == vcsFullManual)) 
	{
		wingsail_update();
	}

	// Calculate Headings during a turn by setting the course dead-down wind breifly to force a gybe, when required.
	// This sets NavData.TurnHDG
	UpdateTurnHeadingV2();
}

void FastLoop(void *) // 50 ms
{
	LED_HeartBeat(13);

	// update the IMU data including compass data
	IMU_Read();
	NavigationUpdate_FastData(); // calculate the true heading
								 // update location data from the GPS
	//if (!UseSimulatedGPS)
	//{
		//GPS_Read();
		GPS_Temp_Read(); // temporary GPS
	//}

	UpdateTargetHeading(); // Target Heading is based on CTS with a Low pass filter

	RC_Read(); // read the RC inputs

	// update steering servo postion based on nav data or RC data, depending on Vessel Command State
	SteeringFastUpdate();

	// get the postion of the wingsail. 
	WingSailAngleSensor_Read(); 

	//if not using simulation, then use real data.
	if (!UseSimulatedVessel)
	{
		WingSail.Angle = WingSailAngleSensor.MagneticAngle;
	}
	else
	{
		WingSail.Angle = simulated_vessel.WingsailAngle;
	}


	// track changes of tack status here, in the fast loop, to ensure fast response to tacking manoeuvre.
	Wingsail_TrackTackChange();
}

void LoggingLoop(void *) 	// 1 second
{
	// This procedure is called periodically to perform periodic data logging to SD Card, Telemetry port or LCD display.
	// V1.0 1/7/2016 John Semmens
	// V1.1 19/10/2018 added test for presence of command port.
	// V1.2 22/12/2018 added reading of Voltage/Current Sensor
	// V1.3 21/2/2019 added update of usage stats object

	// Read the INA219 I2C Voltage/Current Sensor
	read_VI_Measurement();

	// update the usage stats object with the latest individual counters.
	updateUsageTrackingStats();

	SD_Logging(Configuration.SDCardLoggingMask);

	Local_Display_Logging(Configuration.DisplayScreenView);

	// Retrieve any messages from the Bluetooth serial
	if (BTState == BTStateType::Connected)
		BT_CLI_Process_Message(Configuration.BluetoothPort);

	// If we using a simulated Vessel, then perform a periodic update.
	if (UseSimulatedVessel) {
		simulated_vessel.update();
	}
}
	
void SlowLoggingLoop(void *) // 10 minutes
{
	// Avoid running the "10 minute code" if we're still in the first few seconds of start up
	// This procedure seems to get called once in the first few seconds of start up and we don't want it.
	if (millis() > SlowLoggingStartDelay) 
	{
	// log vessel stats to EEPROM
	VesselUsageCounters.intervalCounter++;
	Save_EEPROM_VesselUsage();

	SD_Logging_Event_Usage(Configuration.SDCardLoggingMask);
	}
}

void TelemetryLoop(void*) // 1 second
{
	// pull one message from the queue and send it.
	if (CommandPort)
		ProcessQueue(CommandPort);
}


void StartUpMessage(int SerialPortNumber)
{
	// Send a start-up message to the configured Serial port for communications
	(*Serials[SerialPortNumber]).println(F("MSG,Voyager OS Starting."));
	(*Serials[SerialPortNumber]).print(F("VER,"));
	(*Serials[SerialPortNumber]).print(Version);
	(*Serials[SerialPortNumber]).print(" ");
	(*Serials[SerialPortNumber]).println(VersionDate);
}

void setup()
{
	Serials[0] = &Serial;
	//Serials[1] = &Serial1; these pins are needed for inputs with interrupts for the RC - Input
	Serials[2] = &Serial2;
	Serials[3] = &Serial3;

	// load the configuration from the EEPROM and validate the version of the stored structure
	Load_ConfigValues(0, false); // load config silently, because we don't have the serial comms initialised yet.

	CommandPort = Configuration.CommandPort; // get the configured command port and copy it for use.
	CommandPortBaudRate = Configuration.CommandPortBaudRate;
	// This is because we may override configured value.

	// check the HW Jumper. If the Jumper is on, then override the command serial port to be 3 - Radio Telemetry Channel.
	pinMode(A14, INPUT_PULLUP);
	int HWConfigJumper = digitalRead(A14);
	if (HWConfigJumper == LOW)
	{
		CommandPort = Configuration.SatCommsPort;
		CommandPortBaudRate = Configuration.SatCommsPortBaudRate;
	};

	(*Serials[CommandPort]).begin(CommandPortBaudRate);

	StartUpMessage(CommandPort); // send start up message to serial command port;

	InitPowerManagement(); // start with all off, to ease startup inrush
	TelemetryPowerOn(true);

	// load the configuration from the EEPROM again
	Load_ConfigValues(CommandPort, true); // load config with serial reporting, because we now have the command serial port initialised

	// send start up message to the configured Command Serial Port.
	StartUpMessage(CommandPort);

	GPS_Init(CommandPort); // GPS I2C

	OLED_Init(CommandPort); // OLED Display

	Local_Display_Logging('v'); // initially display the Version information on the LCD. 
					  //This is for a few seconds prior to the configured screen being displayed.

	IMU_Init(CommandPort); // initialise the IMU device for Attitude and Compass BNO080

	init_wearTracking(); // initialise the wear counters

	SailingNavigation_Init(); // set up Target Heading low-pass Filter
	Navigation_Init();// set up True Wind low-pass Filter

	RC_Init(); // Init the Radio Control Input Channels
	Servos_Init(); // Init the Servo Out Channels

	WingSailAngleSensor_Init(CommandPort); // initialise the IMU device for measuring the WingSailAngle

	SteeringPID_Init();

	SchedulerInit();

	// intialise the SD Card logging. Open a new file and write a header record to the log file.
	SD_Logging_Init(CommandPort);
	if (SD_Card_Present) {
		SD_Logging_OpenFile(CommandPort, Configuration.SDCardLoggingMask);
	}	

	//if (CommandPort)
	//	TelemetryLogging_Init(CommandPort,Configuration.TelemetryLoggingMask);

	// display the equipment screen on the OLED
	Local_Display_Logging('e');
	if (SD_Card_Present && WingSailAngleSensor.Initialsed)
	{
		delay(1000); // if no problem, then display for 1 second only
	}
	else
	{
		delay(5000); // longer delay if there's a problem
	}


	// resume mission, if previously in following a mission
	if (StateValues.CommandState==vcsFollowMission)
	{
		// set next waypoint to location specified in the current list command 
		NavData.next_WP = MissionValues.MissionList[StateValues.mission_index].waypoint;
		NavData.next_WP_valid = true;
		NavData.MaxCTE = MissionValues.MissionList[StateValues.mission_index].boundary;
		// insert line here for power settings when it implemented

		if (StateValues.mission_index > 0)
		{
			NavData.prev_WP = MissionValues.MissionList[StateValues.mission_index-1].waypoint;
		}
		else
		{ // mission_index is 0
			// we're powering on, and mission index is zero. so Let's force a re-initialise of the mission to set the starting point here.
			// This is to avoid the 
			StateValues.StartingMission = true;
		}
	}

	Bluetooth_Init(Configuration.BluetoothPort);

	init_VI_Measurement();

	RCPowerOn(true);

	if (CommandPort)
		(*Serials[CommandPort]).println(F("MSG,Voyager OS Pilot Ready."));
}

void loop()
{
	static long prev_loop_time_us; // used for loop timing statistics

	//give the scheduler a chance to act
	SchedulerTick(0,&SlowLoop,  SlowLoopTime);
	SchedulerTick(1,&MediumLoop,  MediumLoopTime);
	SchedulerTick(2,&FastLoop,  FastLoopTime);
	SchedulerTick(3,&LoggingLoop, LoggingLoopTime);
	SchedulerTick(4,&SlowLoggingLoop, SlowLoggingLoopTime);
	SchedulerTick(5,&TelemetryLoop, TelemetryLoopLoopTime);

	// update loop timing statistics
	long micro = micros();
	loop_period_us = micro - prev_loop_time_us;
	prev_loop_time_us = micro;
}

void FlashLed(int FlashCount)
{
	for (int i = 0; i< FlashCount; i++)
	{
		digitalWrite(LEDpin, HIGH);
		delay(50);
		digitalWrite(LEDpin, LOW);
		delay(200);
	}
}