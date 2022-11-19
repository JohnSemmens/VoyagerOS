// Send Telemetry Data back to the Base Station.
// Data of different types and detail, are sent via the serial port in accordance with the specified Telemetry Level.
// 
// TelemetryLogging.h

#ifndef _TELEMETRYLOGGING_h
#define _TELEMETRYLOGGING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "CommandState_Processor.h"
#include "Navigation.h"

// logging masks 
static const int Log_Location  = 0x0001; // loc - Time, Lat, Lon
static const int Log_Attitude  = 0x0002; // att - Time, Pitch, Roll, Compass, Attitude Status
static const int Log_Situation = 0x0004; // sit - Time, BTW, DTW
static const int Log_Waypoints = 0x0008; // way - Time, previous waypoint Lat, Lon, next waypoint Lat, Lon
//static const int Log_Mission   = 0x0010; // mis - Time, mission index, mission command, mission command paramters
static const int Log_Sailing   = 0x0020; // sai - Time, CTS, HDG, WA, BTW, TackTime, CTE , Max CTE
static const int Log_Navigation = 0x0040; // nav - Time, BTW, DTW, RLB, 
static const int Log_System = 0x0080; // sys, looptime
static const int Log_GPS = 0x0100; // sys, GPS state information
static const int Log_Decisions = 0x0200; // dec - Time, Decisions about navigation and sailing.
static const int Log_ServoOut = 0x0400; // svo - Time, 
static const int Log_Voltages = 0x0800; // vlt - Time, 
static const int Log_Usage = 0x1000; // use - Time - Usage Statistics
static const int Log_Environment = 0x2000; // env - Time,
//			spare				0x4000 ;
static const int Log_Messages = 0x8000; // MSG Messages

// enumerated type for Decision Events for the purpose of logging
enum DecisionEventType {
	deIncrementMissionIndex,
	dePastWaypoint,
	deEndOfMission,
	deTackToPort,
	deTackToStarboard,
	deTackToPortRunning,
	deTackToStarboardRunning,
	deChangeCommandState,
	deHoldCourse
};

enum DecisionEventReasonType {
	rPastWaypoint,
	rPastTime,
	rPastDuration,
	rPastBoundary,
	rFavouredTack,
	rNoChange,
	rLimitToSailingCourse,
	rPastLoiterBoundary,
	rUnkown,
	rNone,
	rManualIntervention,
	rApproachingWP
};

//void TelemetryLogging(int CommandPort, word LoggingMask);
//void TelemetryLogging_Init(int CommandPort, word LoggingMask);
//void TelemetryLogging_Event_Decisions(int CommandPort, word LoggingMask);
//void TelemetryLogging_Event_Mission(int CommandPort, word LoggingMask);
//void TelemetryLogging_Event_Usage(int CommandPort, word LoggingMask);

String CommandStateToString(VesselCommandStateType CommandState);
String DecisionEventToString(DecisionEventType DecisionEvent);
String DecisionEventReasonToString(DecisionEventReasonType DecisionEventReason);
String CourseTypeToString(SteeringCourseType CourseType);
String CourseTypeToString(ManoeuvreStateType ManoeuvreState);

#endif

