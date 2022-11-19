// Send Telemetry Data back to the Base Station.
// Data of different types and detail, are sent via the serial port in accordance with the specified Telemetry Level.

// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 3/9/2016 reorganised for logging to use a bit mask.
// V1.3 2/11/2016 added HDG to LOC to aid the Base Station Situation Display
// V1.4 11/11/2016 Added Event based logging, as well as periodoc logging.
// V1.5 29/10/2017 updated for new IMU BNO-055 rather than MPU-9250.
// V1.6 30/10/2017 added IMU status to Attitude
// V1.7 12/11/2017 updated to change BRL to RLB.
// V1.8 25/5/2018 added AWA to SIT

#include "TelemetryLogging.h"
#include "Waypoints.h"
#include "Mission.h"
#include "GPS.h"
#include "Navigation.h"
#include "USFS_IMU.h"
#include "CommandState_Processor.h"
#include "WearTracking.h"

extern NavigationDataType NavData;
extern StateValuesStruct StateValues;
extern long loop_period_us; // microseconds between successive main loop executions
extern IMUStruct myIMU;

extern long GPS_Last_Loc;		// milliseconds since last GPS valid location message
extern long GPS_Last_Message;	// milliseconds since last GPS message
extern bool UseSimulatedVessel; 	// flag to disable the GPS and indicate that the current location is simulated 
extern Time CurrentLocalTime;

extern int WindAngle;	// Wind Angle - degrees positive angles to port, negative angles to starboard: -180 to 0 to +180
//extern File LogFile;
extern HardwareSerial *Serials[];

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern VesselUsageCountersStruct VesselUsageCounters;

//extern double SteeringServoOutput;

extern double PowerSensorV, PowerSensorI;

void TelemetryLogTime(int CommandPort)
{
	// V1.4 22/10/2016 updated to support parameterised serial port
	(*Serials[CommandPort]).print(F("20"));
	(*Serials[CommandPort]).print(CurrentLocalTime.year);
	(*Serials[CommandPort]).print(",");
	(*Serials[CommandPort]).print(CurrentLocalTime.month);
	(*Serials[CommandPort]).print(",");
	(*Serials[CommandPort]).print(CurrentLocalTime.dayOfMonth);
	(*Serials[CommandPort]).print(",");
	(*Serials[CommandPort]).print(CurrentLocalTime.hour);
	(*Serials[CommandPort]).print(",");
	(*Serials[CommandPort]).print(CurrentLocalTime.minute);
	(*Serials[CommandPort]).print(",");
	(*Serials[CommandPort]).print(CurrentLocalTime.second);
	(*Serials[CommandPort]).print(",");
}
//
//void TelemetryLogging(int CommandPort,word LoggingMask)
//{
//	// send telemetry data based on selected mask.
//	// V1.0 4/8/2016 John Semmens
//	// V1.1 3/9/2016 reorganised for logging to use a bit mask.
//	// V1.2 9/10/2016 Added COG,SOG
//	// V1.3 10/10/2016 Added File Size to SYS
//	// V1.4 22/10/2016 updated to support parameterised serial port
//	// V1.5 1/11/2016 changed ATT to use True Hdg
//	// V1.6 2/11/2016 added Heading to LOC
//	// V1.7 29/06/2019 added IMU status to Attitude
//
//	char FloatString[16];
//
//	if (Log_Location & LoggingMask)
//	{
//		// Log_Location  loc - Time, Lat, Lon
//		(*Serials[CommandPort]).print(F("LOC,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL,10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.COG);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.SOG_mps);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.HDG);
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_Attitude & LoggingMask)
//	{
//		// Log_Attitude att - Time, Compass True, Pitch, Roll, Status: system-accel-gyro-mag
//		(*Serials[CommandPort]).print(F("ATT,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(NavData.HDG);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print((int)myIMU.pitch);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print((int)myIMU.roll);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(myIMU.Algorithm_Status);
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_Situation & LoggingMask)
//	{
//		// Log_Situation sit - Time, BTW, DTW, CTE, CDA, AWA
//		(*Serials[CommandPort]).print(F("SIT,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(NavData.BTW);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.DTW);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.CTE);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.CDA);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.AWA);
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_Waypoints & LoggingMask)
//	{
//		// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon, rumb line bearing, max CTE or Boundary
//		(*Serials[CommandPort]).print(F("WAY,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.RLB);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.MaxCTE);
//		(*Serials[CommandPort]).println();
//	}
//
//	//if (Log_Mission & LoggingMask)
//	//{
//	//	// Log_Mission  mis - Time, mission index, command state 
//	//	(*Serials[CommandPort]).print(F("MIS,"));
//	//	TelemetryLogTime(CommandPort);
//	//	(*Serials[CommandPort]).print(StateValues.mission_index);
//	//	(*Serials[CommandPort]).print(",");
//	//	(*Serials[CommandPort]).println(StateValues.CommandState);
//	//}
//
//	if (Log_Sailing & LoggingMask)
//	{
//		// Log_Sailing  sai - Time, CTS, HDG, BTW, AWA,   CTE , Max CTE ,TackTime,
//		(*Serials[CommandPort]).print(F("SAI,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(NavData.CTS);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.HDG);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.BTW);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.AWA);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.CTE);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.MaxCTE);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.TackDuration);
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_Navigation & LoggingMask)
//	{
//		// Log_Navigation nav - Time, BTW, DTW, RLB,
//		(*Serials[CommandPort]).print(F("NAV,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(NavData.BTW);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(NavData.DTW);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).println(NavData.RLB);
//	}
//
//	if (Log_System & LoggingMask)
//	{
//		// Log_System = 0x0080; // sys, looptime, Vessel CommandState
//		(*Serials[CommandPort]).print(F("SYS,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(loop_period_us);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(LogFile.size());
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).println(StateValues.CommandState);
//	}
//
//	if (Log_GPS & LoggingMask)
//	{
//		// Log_GPS = 0x0100; //  GPS state information
//		(*Serials[CommandPort]).print(F("GPS,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(GPS_Last_Loc);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(GPS_Last_Message);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(UseSimulatedGPS);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(GPS_LocationIs_Valid(NavData.Currentloc));
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_ServoOut & LoggingMask)
//	{
//		// Log_ServoOut = 0x0400; // SVO values
//		(*Serials[CommandPort]).print(F("SVO,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(SteeringServoOutput);
//		//(*Serials[CommandPort]).print(",");
//		//(*Serials[CommandPort]).print(DriveMotor.DriveMotorServoOutput);
//		(*Serials[CommandPort]).println();
//	}
//
//	if (Log_Voltages & LoggingMask)
//	{
//		// Log_Voltages = 0x0800; // Voltage values
//		(*Serials[CommandPort]).print(F("VLT,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(0);  // not used
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(0);  // not used
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(PowerSensorV);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(PowerSensorI);
//		(*Serials[CommandPort]).println();
//	}
//
//}

//void TelemetryLogging_Init(int CommandPort, word LoggingMask)
//{
	// send telemetry logging Header based on selected mask.
	// V1.0 3/9/2016 John Semmens
	// V1.1 9/10/2016 Added COG,SOG
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 29/06/2019 added IMU status to Attitude

	//if (Log_Location & LoggingMask)
	//{
	//	// Log_Location  loc - Time, Lat, Lon
	//	(*Serials[CommandPort]).println(F("LOC,YYYY,MM,DD,HH,mm,ss,LAT,LON,COG,SOG,HDG"));
	//}

	//if (Log_Attitude & LoggingMask)
	//{
	//	// Log_Attitude att - Time, Compass True, Pitch, Roll, 
	//	(*Serials[CommandPort]).println(F("ATT,YYYY,MM,DD,HH,mm,ss,HDG,Pitch,Roll,Status"));
	//}

	//if (Log_Situation & LoggingMask)
	//{
	//	// Log_Situation sit - Time, BTW, DTW, CTE, CDA
	//	(*Serials[CommandPort]).println(F("SIT,YYYY,MM,DD,HH,mm,ss,BTW,DTW,CTE,CDA,AWA"));
	//}

	//if (Log_Waypoints & LoggingMask)
	//{
	//	// Log_Waypoints way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon, RLB, MaxCTE
	//	(*Serials[CommandPort]).println(F("WAY,YYYY,MM,DD,HH,mm,ss,LAT,LON,LAT,LON,RLB,MaxCTE"));
	//}

	//if (Log_Mission & LoggingMask)
	//{
	//	// Log_Mission  mis - Time, mission index, command state 
	//	(*Serials[CommandPort]).println(F("MIS,YYYY,MM,DD,HH,mm,ss,MI,CS"));
	//}

	//if (Log_Sailing & LoggingMask)
	//{
	//	// Log_Sailing  sai - Time, CTS, HDG, BTW, AWA,   CTE , Max CTE ,TackTime,
	//	(*Serials[CommandPort]).println(F("SAI,YYYY,MM,DD,HH,mm,ss,CTS,HDG,BTW,AWA,CTE,Max CTE,TT"));
	//}

	//if (Log_Navigation & LoggingMask)
	//{
	//	// Log_Navigation nav - Time, BTW, DTW, RLB,
	//	(*Serials[CommandPort]).println(F("NAV,YYYY,MM,DD,HH,mm,ss,BTW,DTW,RLB"));
	//}

	//if (Log_System & LoggingMask)
	//{
	//	// Log_System = 0x0080; // sys, looptime
	//	(*Serials[CommandPort]).println(F("SYS,YYYY,MM,DD,HH,mm,ss,Loop,CS"));
	//}

	//if (Log_GPS & LoggingMask)
	//{
	//	// Log_GPS = 0x0100; //  GPS state information
	//	(*Serials[CommandPort]).println(F("GPS,YYYY,MM,DD,HH,mm,ss,LAT,LON,Last_Loc,Last_Msg,SimulatedGPS,Loc_Valid"));
	//}

	//if (Log_Decisions & LoggingMask)
	//{
	//	// Log_Decisions = 0x0200; //  
	//	(*Serials[CommandPort]).println(F("DEC,YYYY,MM,DD,HH,mm,ss,MI,CS,DE,DER"));
	//}

	//if (Log_ServoOut & LoggingMask)
	//{
	//	// Log_ServoOut = 0x0400; // SVO values
	//	(*Serials[CommandPort]).println(F("SVO,YYYY,MM,DD,HH,mm,ss,STR,MOT"));
	//}

	//if (Log_Voltages & LoggingMask)
	//{
	//	// Log_Voltages = 0x0800; // VLT values
	//	(*Serials[CommandPort]).println(F("VLT,YYYY,MM,DD,HH,mm,ss,Vm0,Vm1,PSV,PSI"));
	//}

	//if (Log_Usage & LoggingMask)
	//{
	//	// Log_Mission  mis - Time, mission index, command state 
	//	(*Serials[CommandPort]).println(F("USE,YYYY,MM,DD,HH,mm,ss,10min,PortRudder,StarboardRudder,TrimTabCounter"));
	//}
//}

//void TelemetryLogging_Event_Mission(int CommandPort, word LoggingMask)
//{
//	// send telemetry data based on selected mask.
//	if (Log_Mission & LoggingMask)
//	{
//		// Log_Mission  mis - Time, mission index, command state 
//		(*Serials[CommandPort]).print(F("MIS,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(StateValues.mission_index);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).println(CommandStateToString(StateValues.CommandState));
//	}
//}


// Removed TelemetryLogging_Event_Decisions due to suspected corruption of telemetry system, de to async nature of messages
/* 10/1/2020 jrs **************
void TelemetryLogging_Event_Decisions(int CommandPort, word LoggingMask)
{
	if (Log_Decisions & LoggingMask)
	{
		// Log_Decisions  Dec - Time, mission index, command state 
		(*Serials[CommandPort]).print(F("DEC,"));
		TelemetryLogTime(CommandPort);
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CommandStateToString(StateValues.CommandState));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(DecisionEventToString(DecisionEvent));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).println(DecisionEventReasonToString(DecisionEventReason));
	}
}
*/

//void TelemetryLogging_Event_Usage(int CommandPort, word LoggingMask)
//{
//	if (Log_Usage & LoggingMask)
//	{
//		// Log_Usage  mis - Time, mission index, command state 
//		(*Serials[CommandPort]).print(F("USE,"));
//		TelemetryLogTime(CommandPort);
//		(*Serials[CommandPort]).print(VesselUsageCounters.intervalCounter);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(VesselUsageCounters.PortRudderCounter);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(VesselUsageCounters.StarboardRudderCounter);
//		(*Serials[CommandPort]).print(",");
//		(*Serials[CommandPort]).print(VesselUsageCounters.TrimTabCounter);
//		(*Serials[CommandPort]).println();
//	}
//}

String CommandStateToString(VesselCommandStateType CommandState)
{
	// function to return a string version of the CommandState enumnerated type
	// V1.0 21/4/2019 John Semmens

	String CommandStateAsString;
	switch (CommandState)
	{
	case VesselCommandStateType::vcsFollowMission:
		CommandStateAsString = F("FollowMission");
		break;

	case VesselCommandStateType::vcsFullManual:
		CommandStateAsString = F("FullManual");
		break;

	case VesselCommandStateType::vcsIdle:
		CommandStateAsString = F("Idle");
		break;

	case VesselCommandStateType::vcsLoiter:
		CommandStateAsString = F("Loiter");
		break;

	case VesselCommandStateType::vcsPartialManual:
		CommandStateAsString = F("PartialManual");
		break;

	case VesselCommandStateType::vcsResetMissionIndex:
		CommandStateAsString = F("ResetMissionIndex");
		break;

	case VesselCommandStateType::vcsReturnToHome:
		CommandStateAsString = F("ReturnToHome");
		break;

	case VesselCommandStateType::vcsSetHome:
		CommandStateAsString = F("SetHome");
		break;

	case VesselCommandStateType::vcsSteerMagneticCourse:
		CommandStateAsString = F("SteerMagneticCourse");
		break;

	case VesselCommandStateType::vcsSteerWindCourse:
		CommandStateAsString = F("SteerWindCourse");
		break;
	
	default:
		CommandStateAsString = F("Invalid");
 }
	return  CommandStateAsString;
}


String DecisionEventToString(DecisionEventType DecisionEvent)
{
	// function to return a string version of the DecisionEvent enumnerated type
	// V1.0 21/4/2019 John Semmens
	// V1.1 13/6/2019 added deHoldCourse

	String DecisionEventAsString;
	switch (DecisionEvent)
	{
	case DecisionEventType::deEndOfMission:
		DecisionEventAsString = F("EndOfMission");
		break;

	case DecisionEventType::deIncrementMissionIndex:
		DecisionEventAsString = F("IncrementMissionIndex");
		break;

	case DecisionEventType::dePastWaypoint:
		DecisionEventAsString = F("PastWaypoint");
		break;

	case DecisionEventType::deTackToPort:
		DecisionEventAsString = F("toPortTack");
		break;

	case DecisionEventType::deTackToStarboard:
		DecisionEventAsString = F("toStarboardTack");
		break;

	case DecisionEventType::deTackToPortRunning:
		DecisionEventAsString = F("toPortTackRunning");
		break;

	case DecisionEventType::deTackToStarboardRunning:
		DecisionEventAsString = F("toStarboardTackRunning");
		break;

	case DecisionEventType::deChangeCommandState:
		DecisionEventAsString = F("ChangeCommandState");
		break;

	case DecisionEventType::deHoldCourse:
		DecisionEventAsString = F("HoldCourse");
		break;		

	default:
		DecisionEventAsString = F("Invalid");
	}
	return DecisionEventAsString;
}

String DecisionEventReasonToString(DecisionEventReasonType DecisionEventReason)
{
	// function to return a string version of the DecisionEvent enumnerated type
	// V1.0 21/4/2019 John Semmens
	// V1.1 13/6/2019 added rApproachingWP

	String DecisionEventReasonAsString;
	switch (DecisionEventReason)
	{
	case DecisionEventReasonType::rFavouredTack:
		DecisionEventReasonAsString = F("FavouredTack");
		break;

	case DecisionEventReasonType::rNoChange:
		DecisionEventReasonAsString = F("NoChange");
		break;

	case DecisionEventReasonType::rPastBoundary:
		DecisionEventReasonAsString = F("PastBoundary");
		break;

	case DecisionEventReasonType::rPastDuration:
		DecisionEventReasonAsString = F("PastDuration");
		break;

	case DecisionEventReasonType::rPastTime:
		DecisionEventReasonAsString = F("PastTime");
		break;

	case DecisionEventReasonType::rPastWaypoint:
		DecisionEventReasonAsString = F("PastWaypoint");
		break;

	case DecisionEventReasonType::rUnkown:
		DecisionEventReasonAsString = F("Unkown");
		break;

	case DecisionEventReasonType::rLimitToSailingCourse:
		DecisionEventReasonAsString = F("LimitToSailingCourse");
		break;

	case DecisionEventReasonType::rPastLoiterBoundary:
		DecisionEventReasonAsString = F("PastLoiterBoundary");
		break;

	case DecisionEventReasonType::rNone:
		DecisionEventReasonAsString = F("None");
		break;

	case DecisionEventReasonType::rManualIntervention:
		DecisionEventReasonAsString = F("ManualIntervention");
		break;

	case DecisionEventReasonType::rApproachingWP:
		DecisionEventReasonAsString = F("ApproachingWP");
		break;

	default:
		DecisionEventReasonAsString = F("Invalid");
	}
	return DecisionEventReasonAsString;
}

String CourseTypeToString(SteeringCourseType CourseType)
{
	// function to return a string version of the CourseType enumnerated type
	// V1.0 5/5/2019 John Semmens
	// V1.1 10/1/2021 filled out additional new running values.

	String CourseTypeAsString;

	switch (CourseType)
	{
	case SteeringCourseType::ctDirectToWayPoint:
		CourseTypeAsString = F("DirectToWayPoint");
		break;

	case SteeringCourseType::ctPortTack:
		CourseTypeAsString = F("PortTack");
		break;

	case SteeringCourseType::ctStarboardTack:
		CourseTypeAsString = F("StarboardTack");
		break;


	case SteeringCourseType::ctPortTackRunning:
		CourseTypeAsString = F("PortTackRun");
		break;

	case SteeringCourseType::ctStarboardTackRunning:
		CourseTypeAsString = F("StbdTackRun");
		break;

	default: 
		CourseTypeAsString = F("Invalid");
	}

	return CourseTypeAsString;
}


String CourseTypeToString(ManoeuvreStateType ManoeuvreState)
{
	// function to return a string version of the ManoeuvreState enumnerated type
	// V1.0 29/5/2019 John Semmens
	String ManoeuvreStateAsString;

	switch (ManoeuvreState)
	{
	case ManoeuvreStateType::mstCommence:
			ManoeuvreStateAsString = F("Commence");
		break;

	case ManoeuvreStateType::mstComplete:
		ManoeuvreStateAsString = F("Complete");
		break;

	case ManoeuvreStateType::mstNone:
		ManoeuvreStateAsString = F("None");
		break;

	case ManoeuvreStateType::mstRunning:
		ManoeuvreStateAsString = F("Running");
		break;
	default:
		ManoeuvreStateAsString = F("Invalid");
	}

	return ManoeuvreStateAsString;
}