// 
// 
// 

#include "CommandState_Processor.h"
#include "RadioControl.h"
#include "Navigation.h"
#include "configValues.h"
#include "Loiter.h"
#include "location.h"
#include "AP_Math.h"
#include "TelemetryLogging.h"
#include "SDCardLogFile.h"
#include "Mission.h"

extern StateValuesStruct StateValues;
extern RC_IN_Type RC_IN;
extern NavigationDataType NavData;
extern configValuesType Configuration;
extern LoiterStruct LoiterData;
extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int DecisionEventValue; 
extern int CommandPort;
extern MissionValuesStruct MissionValues;	// structure holding mission details

VesselCommandStateType GetCommandStateFromRC_Command_Switch(int RCSwitchPos)
{
	// Function to translate a RC-In Switch value to a Command State.
	// This allows an RC Command Channel to be used to control the command state fo the vessel.
	// V1.0 4/10/2016 John Semmens

	VesselCommandStateType cs;

	switch (RCSwitchPos)
	{
	case 0: // invalid, hence idle.
		cs = vcsIdle;
		break;

	case 1:
		cs = vcsFullManual;
		break;

	case 2:
		cs = vcsPartialManual;
		break;

	case 3:
		cs = vcsFollowMission;
		break;
	case 4:
		cs = vcsReturnToHome;
		break;

	case 5:
		cs = vcsSetHome;
		break;

	case 9: // no RC Signal
		cs = Configuration.FailsafeCommandState;
		//cs = vcsLoiter; // set the failsafe to loiter while testing.
		break;

	default:
		cs = vcsIdle;
	}
	return cs;
}

void CommandState_Processor(void)
{
	// Process Command state change requests
	// change requests come from the RC Command Channel - Switch Position.
	// also from the Command Line Interpreter.
	// This is called from the medium loop

	// V1.1 7/4/2019 updated to better handle entering "loiter Here" by factoring in the current tack.

	static VesselCommandStateType prev_rc_cs;
	VesselCommandStateType rc_cs = GetCommandStateFromRC_Command_Switch(RC_IN.RC_Command_Switch_Position);

	static VesselCommandStateType prev_cs;

	// if there's a change in the command state coming from the Radio Control input then apply the change.
	if (prev_rc_cs != rc_cs)
	{
		prev_rc_cs = rc_cs;
		StateValues.CommandState = rc_cs;
	}

	// look for changes in command state, against the previous state
	if (prev_cs != StateValues.CommandState)
	{
		prev_cs = StateValues.CommandState;
		
		// process the new command state
		switch (StateValues.CommandState)
		{
		case vcsResetMissionIndex:
			// Reset Mission Index
			StateValues.mission_index = 0;
			StateValues.StartingMission = true;
			break;

		case vcsSetHome:
			// get the current location and set the home location
			StateValues.home = NavData.Currentloc;
			StateValues.home_is_set = true;
			break;

		case vcsLoiter:
				// get the current location and apply as the loiter location 
				LoiterData.loiterCentreLocation = NavData.Currentloc;
				LoiterData.loiterLocationPrevious = NavData.Currentloc;

				
				// set the radius
				LoiterData.LoiterRadius = Configuration.LoiterRadius;

				// if currently on starboard tack, then commence loitering to the port side
				if (NavData.AWA >0)  // positive is starboard tack
				{
					// set loiter wp to portside. We do this by getting the centre point and then calculating the location
					// of the port side waypoint which is 90 degrees to port from the True Wind Angle.
					LoiterData.LoiterState = LoiterStateType::lsPortSide;
					LoiterData.LoiterLocationNext = NavData.Currentloc;
					location_update(LoiterData.LoiterLocationNext, wrap_360(NavData.TWD - 90), float(LoiterData.LoiterRadius));
					NavData.next_WP = LoiterData.LoiterLocationNext; // added for Base Station Display only
				}
				else
				{
					// set loiter wp to starboardside. We do this by getting the centre point and then calculating the location
					// of the starboard side waypoint which is 90 degrees to starboard from the True Wind Angle.
					LoiterData.LoiterState = LoiterStateType::lsStarboardSide;
					LoiterData.LoiterLocationNext = NavData.Currentloc;
					location_update(LoiterData.LoiterLocationNext, wrap_360(NavData.TWD + 90), float(LoiterData.LoiterRadius));
					NavData.next_WP = LoiterData.LoiterLocationNext; // added for Base Station Display only
				}
			break;

		case vcsReturnToHome:
			// if home location is set, then set the next waypoint as the home location
			if (StateValues.home_is_set)
			{
				NavData.prev_WP = NavData.Currentloc;
				NavData.next_WP = StateValues.home;
				NavData.MaxCTE = Configuration.DefaultMaxCTE;
				NavData.next_WP_valid = true;
			}

		// no action here on these.
		case vcsSteerMagneticCourse:
			DecisionEventValue = StateValues.SteerCompassBearing;
			break;
		case vcsSteerWindCourse:
			DecisionEventValue = StateValues.SteerWindAngle;
			break;
		case vcsFollowMission:
		case vcsIdle:
		case vcsFullManual:
		case vcsPartialManual:
			break;

		default:;
		}

		// log the new command state
		DecisionEvent = DecisionEventType::deChangeCommandState;
		DecisionEventReason = DecisionEventReasonType::rManualIntervention;
		SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
	}

	// Failsafe timeout on Steering Wind Cource or Compass Course, when there's no Vessel Command State message for the timeout period.
	// when it timeso ut, it will change to Return to home, provided Home is set.
	if ((StateValues.CommandState == vcsSteerMagneticCourse) || (StateValues.CommandState == vcsSteerWindCourse))
	{
		if ((((millis() - MissionValues.MissionCommandStartTime) / 1000) >= Configuration.RTHTimeManualControl)
			&& (StateValues.home_is_set))
		{
			// if home is set then go home AND if time exceeds 
			StateValues.CommandState = VesselCommandStateType::vcsReturnToHome;

			DecisionEvent = DecisionEventType::deChangeCommandState;
			DecisionEventReason = DecisionEventReasonType::rPastTime;
			SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
		}
	}

}

