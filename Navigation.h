// Navigation.h
// This considers the Next Waypoint and the Previous Waypoint and the permitted Cross Track Error (Boundary).
// It then calculates the desired Distance and Bearing to Waypoint.
// It also considers whether the current leg of the mission has been completed.

// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 12/9/2016 added enumerated type for current steering method.
// V1.3 9/11/2016 added prev_WP_valid
// V1.4 12/11/2017 updated to change BRL to RLB.
// V1.5 14/4/2018 added GPS directly provided course and speed to Navdata object
// V1.6 20/6/2020 added VMG and VMC, SOG_Avg and COG_Avg
// V1.7 2/1/2021 added Laylines for running to the NavData structure
//				 added Port and Starboard Tack Running to SteeringCourseType enum

#ifndef _NAVIGATION_h
#define _NAVIGATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Waypoints.h"

enum PointOfSailType {
	psNotEstablished,			 // sailing state has not been established yet
	psPortTackBeating,	
	//ssPortTackCompass,		 // steering by Compass
	psPortTackRunning,		
	psStarboardTackBeating,
	//ssStarboardTackCompass,	 // steering by Compass
	psStarboardTackRunning,	
};

enum SteeringCourseType {
	ctNotEstablished,	//  the Course has not been established yet
	ctDirectToWayPoint, // the Course to the Waypoint is Sailable, so the course is direct to Way Point.
	ctPortTack,			// the Course to the Waypoint is NOT Sailable, so this port tack while tacking to Waypoint
	ctStarboardTack,		// the Course to the Waypoint is NOT Sailable, so this Starboard tack while tacking to Waypoint
	ctPortTackRunning,			// the Course to the Waypoint is NOT Sailable, so this port tack while Running to Waypoint
	ctStarboardTackRunning		// the Course to the Waypoint is NOT Sailable, so this Starboard tack while Running to Waypoint
};

enum ManoeuvreType {
	mtNotDefined,	// don't specify how thw turn is to be perform. This turn in shortest direction to make the new course
	mtTack,			// Tack through the wind to get to the new course
	mtGybe			// Gybe to get to the new course.
};

enum ManoeuvreStateType {	// used in a state machine to manage a turn manoeuvre
	mstNone,		// not in a manoeuvre
	mstCommence,	// commenced manoeuvre 
	mstRunning,		// acheived a verified running state
	mstComplete		// manoeuvre Completed 
};

struct NavigationDataType {
	 Location Currentloc;	// Current GPS Location
	 long CurrentLocTimeStamp; // time in millis when the current location was measured.
	 Location prev_WP;		// The location of the previous waypoint.  Used for track following and altitude ramp calculations
	 Location next_WP;		// The location of the current/active waypoint.  Used for track following
	 int MaxCTE;			// Maximum Cross Track Error - Metres -  (Boundary).

	 long DTW;			 // Distance to Waypoint - metres
	 int BTW;			 // Bearing to Waypoint - Degrees
	 bool PastWP;		 // past the Waypoint -  True/False
	 int RLB;			 // RLB Rumb Line Bearing - Degrees - Angle.
	 int CDA;			 // Course deviation Angle -Degrees - angle between the rumb line and bearing to waypoint.
						 // +Ve CDA means Starboard Side of Course.
	 int CTE;			 // Cross Track Error - metres -- same sign as CDA. +Ve CTE means Starboard Side of Course.
	 int CTE_Correction; // Steering adjustment angle to compenstate for CTE. This is added into the CTS when steering Direct to Waypoint.

	 bool next_WP_valid; // Flag to indicate that we have an established next waypoint.

	 float SOG_mps;		 // Speed Over Ground -- metres/second
	 float SOG_knt;		 // Speed Over Ground -- Knots
	 int COG;			 // Course Over Ground - Degrees - True
	 
	 int HDG;			// True Heading -Degrees -- Derived from compass with Variation applied.

	 // Sailing Navigation Global Variables
	 int CTS;		     // Course To Steer (CTS) - Angle -Degrees - this is the calculated course to steer
						 // CTS will instantaneously change to a new bearing, when its time to tack
	 int TurnHDG;		 // Turn Heading used to manage the Target Heading during a Turn Manoeuvre.
						 // This is used to force a turn to be gybe, rather than taking the shortest heading change.
	 int TargetHDG;		 // Target Heading is low-pass filtered version of TurnHDG (was CTS). This is used for the actual helming.
						 // When CTS instantaneously changes heading, for tack or a gybe, The Target Heading will change more slowly,
						 // thereby providing providing guidance through the manoeuvre. 
						 // the low-pass filter constant will provide a control over the turn-rate.

	 int TargetAWA;		 // Target Apparent Wind Angle. This is the desired angle to steer off the wind. A Positive angle is Starboard Tack.
						 // This compared periodically to the AWA to determine the steering error, which used to determine the compass 
						 // course to steer.

	 // Environment 
	 int AWA;		// Apparent Wind Angle - degrees; positive angles to starboard, negative angles to port: -180 to 0 to +180
	 int AWD;		// Apparent Wind Direction - 0 degrees is a North Wind. 90 degrees is East Wind. Raw unadjusted Wind Direction 
	 int TWD;		// True Wind Direction - degrees; 0 degrees is a North Wind. 90 degrees is East Wind. This is an adjusted version of AWD
	 int TWD_Offset; // This is fudge factor to hack a TWD, without measuring Wind Speed.
	 float TWS;		// True Wind Speed metres per second

	 long TackDuration; // seconds. Duration of current Tack
	 long TimePastFavouredTack; // Time since the we've moved past the point of being on the favoured tack.

	 PointOfSailType PointOfSail; // which tack, and what point of sail.
	 SteeringCourseType CourseType; // 

	 int WindAngleToWaypoint;	// This is wind angle if the vessel was heading directly to the Waypoint.
								// This is subject to errors related to True versus Apparent Wind.

	 bool IsBTWSailable; // determine if the BTW is sailable or not. i.e. can the course be laid.


	 int PortLayline; // Steering Course when closehauled Port Tack.
	 int StarboardLayline; // Steering Course when closehauled Starboard Tack.
	 int PortLaylineRunning; // Steering Course when Running on Port Tack.
	 int StarboardLaylineRunning; // Steering Course when Running on Starboard Tack.
	 SteeringCourseType FavouredTack; // This is the tack that yields a course which is closest ot the BTW
	 ManoeuvreType Manoeuvre; // this describes how the course changes should be performed. i.e. tack or gybe or don't specify.
	 ManoeuvreStateType  ManoeuvreState; // this describes the current state of the Manoeuvre


	 int HDG_Mag;			// Magnetic Heading, not corrected by the GPS COG
	 int HDG_Err;			// Heading Error derived from GPS data
	 int ROLL_Avg;			// dampened Roll value passed through a low pass filter

	 float COG_Avg;			// dampened COG
	 float SOG_Avg;			// dampened SOG
	 float VMG;				// Velocity Made Good to windward
	 float VMC;				// Velocity Made Good on Course

	 long DTH;			 // Distance to Home - metres -- valid only if home is set
	 int BTH;			 // Bearing to Home - Degrees -- valid only if home is set
};

void NavigationUpdate_SlowData(void);
void NavigationUpdate_MediumData(void);
void NavigationUpdate_FastData(void);

void Navigation_Init(void);
void GetTrueWind(void);
void GetApparentWind(void);

int get_CTE_Correction(NavigationDataType NavData);

void UpdateTurnHeadingV2(void);

int AWA_Calculated(int CTS, int TWD);

#endif

