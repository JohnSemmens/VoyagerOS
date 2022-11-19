// 
// module for calculating the loitering movements for  sailing 
// This is implemented as loitering state machine
// Sailing Loitering consists of reaching back and forth across the loiter circle.

// V1.0 1/2/2019 Initial Version of Sailing Loitering
// V1.1 14/2/2019 added Gybe Turn to Loitering
// V1.2 24/4/2019 added event logging 

// consider reducing power of the sail while loitering

#include "Loiter.h"
#include "configValues.h"
#include "location.h"
#include "Navigation.h"
#include "AP_Math.h"
#include "SailingNavigation.h"
#include "TelemetryLogging.h"
#include "SDCardLogFile.h"

extern NavigationDataType NavData;
extern configValuesType Configuration;
extern LoiterStruct LoiterData;
extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int CommandPort;

int LoiterCalcCTS(void)
{   // (called at 5s intervals)
	// calc bearing to loiter point
	LoiterData.BTW = get_bearing(NavData.Currentloc, LoiterData.LoiterLocationNext);
	int CTS = LimitToSailingCourse(LoiterData.BTW);

	// get distance to loiter point
	LoiterData.DTW = get_distance(NavData.Currentloc, LoiterData.LoiterLocationNext);

	LoiterData.PastWP = location_passed_point(NavData.Currentloc, LoiterData.loiterLocationPrevious, LoiterData.LoiterLocationNext);

	// Process the Loiter State machine
	switch (LoiterData.LoiterState)
	{
	case lsApproach:
	case lsStarboardSide:
		// sail to loiter point
		if (LoiterData.PastWP)
		{
			// if we have passed point then switch to lsPortSide
			LoiterData.LoiterState = lsPortSide;
			LoiterData.PastWP = false;
			// set loiter wp to portside. We do this by getting the centre point and then calculating the location
			// of the port side waypoint which is 90 degrees to port from the True Wind Angle.
			LoiterData.LoiterLocationNext = LoiterData.loiterCentreLocation;
			location_update(LoiterData.LoiterLocationNext, wrap_360(NavData.TWD - 90), float(LoiterData.LoiterRadius));
			LoiterData.loiterLocationPrevious = NavData.Currentloc;
			NavData.next_WP = LoiterData.LoiterLocationNext; // added for Base Station Display only
			NavData.Manoeuvre = ManoeuvreType::mtGybe;
			NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;

			DecisionEvent = DecisionEventType::deTackToPort;
			DecisionEventReason = DecisionEventReasonType::rPastLoiterBoundary;
			SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
		}
		break;

	case lsPortSide:
		// sail to lsPortSide loiter point
		if (LoiterData.PastWP)
		{
			// if we have passed point then switch to lsStarboardSide
			LoiterData.LoiterState = lsStarboardSide;
			LoiterData.PastWP = false;
			// set loiter wp to StarboardSide. We do this by getting the centre point and then calculating the location
			// of the Starboard Side waypoint which is 90 degrees to starboard from the True Wind Angle.
			LoiterData.LoiterLocationNext = LoiterData.loiterCentreLocation;
			location_update(LoiterData.LoiterLocationNext, wrap_360(NavData.TWD + 90), LoiterData.LoiterRadius);
			LoiterData.loiterLocationPrevious = NavData.Currentloc;
			NavData.next_WP = LoiterData.LoiterLocationNext; // added for Base Station Display only
			NavData.Manoeuvre = ManoeuvreType::mtGybe;
			NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;

			DecisionEvent = DecisionEventType::deTackToStarboard;
			DecisionEventReason = DecisionEventReasonType::rPastLoiterBoundary;
			SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
		}
		break;

	case lsNotLoitering:
		break;

	default:;
	}

	return CTS;
};

