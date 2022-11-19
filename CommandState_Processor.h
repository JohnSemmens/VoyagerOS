// CommandState_Processor.h

#ifndef _COMMANDSTATE_PROCESSOR_h
#define _COMMANDSTATE_PROCESSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Waypoints.h"

// this is the current state of the vessel. This would be generally controlled externally (manually) from either an RC link, or a Serial link.
enum VesselCommandStateType {
	vcsIdle,				// idle state, feathered settings for sails.
	vcsFullManual,			// The vessel is under full manual command via RC. i.e. steering and sail
	vcsPartialManual,		// The vessel is under partial manual command via RC. i.e. steering only, with auto-sail control
	vcsFollowMission,		// The vessel is under automatic control following the mission list.
	vcsSteerMagneticCourse,	// The vessel is steering a course relative to the compass.
	vcsSteerWindCourse,		// The vessel is steering a course relative to the wind.
	vcsReturnToHome,		// return to the preset home location
	vcsSetHome,				// set home location
	vcsResetMissionIndex,	// Reset the Mission Index, to force a restart of the mission.
	vcsLoiter				// Loiter here
};

// this structure holds the current state of the vessel.
// it is intended to allow the state of the vessel to be recovered if the computer is restarted.
// That is: you don't want the vessel returning home to repeat the whole mission if the computer is restarted half way through a mission.
// You generally want it to resume from the current mission position.
struct StateValuesStruct {
	VesselCommandStateType CommandState;	// current command state of the vessel  (i.e. manual control, missionList)
	int mission_index;				// pointer to the current mission command. zero means first command of mission.
	bool StartingMission;		// flag to signal the start of a mission.
								// This is primarily to create a starting transition event while the mission index remains at zero.

	Location home;					// The home location used for RTL.  
	bool     home_is_set;			//
	int SteerWindAngle;				// degrees. used only in conjunction with the Vessel command to steer a wind course
	int SteerCompassBearing;		// degrees. used only in conjunction with the Vessel command to steer a compass course
};

VesselCommandStateType GetCommandStateFromRC_Command_Switch(int RCSwitchPos);
void CommandState_Processor(void); 

#endif

