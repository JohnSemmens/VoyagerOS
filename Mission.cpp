// Mission
// This provides the structures and function to process a mission
// including advancing the mission index when each step of the mission is completed.

// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 28/4/2018 updated to reorganise design and improve quality.
// V1.3 16/7/2019 adding the new Mission command for Steering a Wind Angle

#include "Mission.h"
#include "configValues.h"
#include "Navigation.h"
#include "CommandState_Processor.h"
#include "TelemetryLogging.h"
#include "SDCardLogFile.h"
#include "GPS.h"
#include "Loiter.h"
#include "PowerManagement.h"

extern LoiterStruct LoiterData;

extern NavigationDataType NavData;
extern int CommandPort;

extern StateValuesStruct StateValues;
extern MissionValuesStruct MissionValues;
extern Time CurrentLocalTime;
extern configValuesType Configuration;

extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis

void MissionUpdate(void) {
	// This is called from the slow loop.
	// Review if position in mission list needs to be changed
	// V1.0 7/8/2016 John Semmens
	// V1.1 10/11/2016 updated to provide a way to reset the mission back to zero.

	// check if we're following a mission and the mission has size
	// and verify that the location is valid
	if ( (StateValues.CommandState == VesselCommandStateType::vcsFollowMission)
		    && (MissionValues.mission_size > 0) 
			&& GPS_LocationIs_Valid(NavData.Currentloc) )
		{
			// check if we are just starting and intialise the first mission step parameters
			if (StateValues.StartingMission == true)
				{
				StateValues.StartingMission = false;

				// start command timer. Record the start time of each new command	
				MissionValues.MissionCommandStartTime = millis();

				//set up the next waypoint
				UpdatePrevNextWP();
				//initialise prev_wp with current loc
				NavData.prev_WP = NavData.Currentloc;

				// Log Mission Step Details
				SD_Logging_Event_MissionStep(StateValues.mission_index);
				}
			
			// increment Mission index if step complete
			if (IsMissionStepComplete())
			{
				StateValues.mission_index++;
				// save the state
				Save_EEPROM_StateValues();

				// Update Power Control
				PowerControl(MissionValues.MissionList[StateValues.mission_index].controlMask);

				// start command timer. Record the start time of each new command	
				MissionValues.MissionCommandStartTime = millis(); 
					
				// Log Mission Step Details
				SD_Logging_Event_MissionStep(StateValues.mission_index);

				//set up the next waypoint
				UpdatePrevNextWP();
			}
		}
}

bool IsMissionStepComplete(void) {
	// check if the current mission step is complete and increment Mission index if required
	// return true, if the current step is complete, false otherwise.

	// V1.0 7/8/2016 John Semmens
	// V1.1 13/1/2017 Updated to improve protection of the mission index and prevent out of bounds access.
	// V1.2 28/4/2018 refactored and simplified design.
	// V1.3 1/8/2019 ensure RC Rx and Telemetry are powered up at end of mission.

	int MinutesPastMidnight;
	bool StepComplete = false;

	// if we are not yet on the last step of the mission, then proceed
	if (StateValues.mission_index < MissionValues.mission_size)
	{
		// get the command from the list and decide what to do.
		MissionCommandType mc = MissionValues.MissionList[StateValues.mission_index].cmd;

		switch (mc)
		{
		case ctGotoWaypoint:
		case ctReturnToHome:
			// check if past current waypoint and if so, advance mission index.
			if (NavData.PastWP)
			{
				StepComplete = true;

				// log the event
				DecisionEvent = DecisionEventType::deIncrementMissionIndex;
				DecisionEventReason = DecisionEventReasonType::rPastWaypoint;
				SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
			}
			break;

		case ctLoiter:
		case ctSteerWindCourse:
			// check if past duration and if so, advance mission index.
			// mission command duration is expressed in minutes.
			// perform the test using seconds. There was an overflow occuring when milliseconds was used
			if (((millis() - MissionValues.MissionCommandStartTime)/1000) >= (MissionValues.MissionList[StateValues.mission_index].duration * 60))
			{
				StepComplete = true;

				// log the event
				DecisionEvent = DecisionEventType::deIncrementMissionIndex;
				DecisionEventReason = DecisionEventReasonType::rPastDuration;
				SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
			}
			break;

		case ctLoiterUntil:
			// check if past Time Of Day and if so, advance mission index.
			// Time Of Day is expressed in minutes past midnight. e.g. 6am = 360 minutes past midnight
			// perform the test using minutes.
			MinutesPastMidnight = CurrentLocalTime.hour * 60 + CurrentLocalTime.minute;
			if (MinutesPastMidnight >= MissionValues.MissionList[StateValues.mission_index].duration)
			{
				StepComplete = true; 
				
				// log the event
				DecisionEvent = DecisionEventType::deIncrementMissionIndex;
				DecisionEventReason = DecisionEventReasonType::rPastTime;
				SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
			}
			break;

		default:;
		}
	}
	else
	{
		// we're at end of mission
		if (StateValues.home_is_set)
		{
			// if home is set then go home.
			StateValues.CommandState = VesselCommandStateType::vcsReturnToHome;
		}
		else
		{
			// if home is not set then loiter here.
			StateValues.CommandState = VesselCommandStateType::vcsLoiter;
		}

		// make sure power is on for the radios at end of mission
		RCPowerOn(true);
		TelemetryPowerOn(true);

		// log the event
		DecisionEvent = DecisionEventType::deEndOfMission;
		SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
	}

	return StepComplete;
};

void UpdatePrevNextWP(void)
{
	// We've moved to the next mission step, so update the next and previous waypoints 
	// V1.0 4/8/2016 John Semmens
	// V1.1 25/1/2017 added Loitering within the Mission Commands
	// V1.2 28/4/2018 refactored and simplified design.

	// set previous waypoint to be next waypoint, or, if not valid, then use current loc  
	if (NavData.next_WP_valid) {
		NavData.prev_WP = NavData.next_WP;
	}
	else {
		NavData.prev_WP = NavData.Currentloc;
	}

	// get the command from the list and decide what to do.
	MissionCommandType mc = MissionValues.MissionList[StateValues.mission_index].cmd;

	switch (mc)
	{
	case ctLoiter:
	case ctLoiterUntil:
		// set next waypoint to location specified in the current list command 
		NavData.next_WP = MissionValues.MissionList[StateValues.mission_index].waypoint;
		NavData.MaxCTE = MissionValues.MissionList[StateValues.mission_index].boundary;
		NavData.next_WP_valid = true;

		// set up the loiter variables
		// set loiterOuterLocation to be previous waypoint
		LoiterData.loiterLocationPrevious = NavData.prev_WP;
		// set loiter point to be next waypoint
		LoiterData.loiterCentreLocation = NavData.next_WP;
		// set Loiter radius
		LoiterData.LoiterRadius = MissionValues.MissionList[StateValues.mission_index].boundary;
		// set loiter state to approach
		LoiterData.LoiterState = lsApproach;
		break;

	case ctGotoWaypoint:
		// set next waypoint to location specified in the current list command 
		NavData.next_WP = MissionValues.MissionList[StateValues.mission_index].waypoint;
		NavData.MaxCTE = MissionValues.MissionList[StateValues.mission_index].boundary;
		NavData.next_WP_valid = true;
		break;

	case ctReturnToHome:
		// set next Waypoint to be home location.
		NavData.next_WP = StateValues.home;
		// NavData.MaxCTE = ?? // perhaps leave unchanged from previous setting
		NavData.next_WP_valid = true;
		break;

	case ctSteerWindCourse:
		// steer a wind course for a duration.
	    NavData.next_WP_valid = true; 	// do we need this ?? YES otherwise other parts of follow mission don't work.
		StateValues.SteerWindAngle = MissionValues.MissionList[StateValues.mission_index].SteerAWA;
		break;

	default:;
	}

}

void set_next_WP_for_display()
{
	// update the next Waypoint outside of the normal Following mission
	// this is helpful for display on the Voyager Base Station.

	// if we are not following a mission, and the mission has size
	// and the mission index is within  the mission list
	if ((StateValues.CommandState != VesselCommandStateType::vcsFollowMission)
		&& (MissionValues.mission_size > 0)
		&& (StateValues.mission_index <= MissionValues.mission_size)
		)
	{
		// make sure the next waypoint is set, even though we're currently set to follow mission.
		// this is helpful for display on the Voyager Base Station.

		// set next waypoint to location specified in the current list command 
		NavData.next_WP = MissionValues.MissionList[StateValues.mission_index].waypoint;
		NavData.MaxCTE = MissionValues.MissionList[StateValues.mission_index].boundary;
		//	NavData.next_WP_valid = true;
	}
}