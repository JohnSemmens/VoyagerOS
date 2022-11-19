// Mission.h

#ifndef _MISSION_h
#define _MISSION_h

#include "Waypoints.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// These are Mission commands. They are part of the pre-programmed mission.
enum MissionCommandType {
	ctGotoWaypoint,	// goto waypoint, stay within boundary distance of the rumb line (max CTE)
	ctLoiter,		// loiter at the waypoint, for duration, and within boundary distance
	ctLoiterUntil,	// loiter at the waypoint, until a time of day, and within boundary distance
	ctReturnToHome,	// return to the preset home location
	ctSteerWindCourse // Steer a wind course.
};

// This describes the individual mission commands used in the mission command array.
struct MissionCommand {
	MissionCommandType cmd;
	Location waypoint;
	int boundary;	  // metres
	int controlMask; // bit mask to control behaviour on each mission step. e.g. power control
	int duration;	  // minutes
	int SteerAWA; // degrees -- used for steering wind angle only.
	int TrimTabAngle; // degrees -- used for steering wind angle only.
};


// set size of array of Mission Commands 
static const int MaxMissionCommands = 30;

struct MissionValuesStruct {
	MissionCommand MissionList[MaxMissionCommands];
	int mission_size;				// size of the current mission. zero means no mission. 
	unsigned long MissionCommandStartTime;		// start time in ms for the current command.
};

void MissionUpdate(void);
bool IsMissionStepComplete(void);
void UpdatePrevNextWP(void);

void set_next_WP_for_display(void);

#endif

