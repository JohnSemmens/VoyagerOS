// This is where the sailing course is decided.
// The course to be steered is established here, including the decisions to tack.
// The actual course to be steered is maintained in the Course To Steer (CTS).

// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 10/2/2019 added tacking concepts into CalculateSailingCTS
// V1.3 11/2/2019 addded decision logging for tacks and tack choices.
// V1.4 12/2/2019 fix logic reversal in tacking decisions.
// V1.5 3/1/2021 added in downwind tacking functionality.

#include "SailingNavigation.h"
#include "Navigation.h"
#include "Filters.h"
#include "configValues.h"
#include "CommandState_Processor.h"
#include "Loiter.h"
#include "Mission.h"
#include "location.h"
#include "TelemetryLogging.h"
#include "SDCardLogFile.h"

extern NavigationDataType NavData;
extern StateValuesStruct StateValues;
extern configValuesType Configuration;
extern MissionValuesStruct MissionValues;
extern DecisionEventType DecisionEvent;				// used in event based logging and diagnosis
extern DecisionEventReasonType DecisionEventReason;	// used in event based logging and diagnosis
extern int CommandPort;

LowPassAngleFilter TargetHeadingFilter;

void SailingNavigation_Init(void)
{
	// initialize values for the Sailing Navigiation processes.
	// This should be called at start up.
	TargetHeadingFilter.FilterConstant = Configuration.TargetHeadingFilterConstant;
}

void UpdateCourseToSteer(void)
{	// called in the slow loop -- about 5 seconds
	// Set a sailing course to steer to get to the waypoint, making appropriate tacking decisions
	// and remaining within the set boundaries of the sailing leg.
	// Review current situation and decide whether a tack is required or not.
	// V1.1 12/6/2019 updated to prevent course changes while in the waypoint boundary circle
	//				change tests for boundary to ensure that they are not sensitive to CTE sign. 
	// V1.2 13/6/2019 added test for being within boundary distance of the WP and then Hold course with DEC logging.
	// V1.3 16/7/2019 adding the new Mission command for Steering a Wind Angle
	// V1.4 19/7/2019 changed DTW Course Hold Radius to use parameter and not boundary


	// get the current command from the list and decide what to do.
	MissionCommandType mc = MissionValues.MissionList[StateValues.mission_index].cmd;

	switch (StateValues.CommandState)
	{
	case vcsFollowMission:

		switch (mc)
		{
		case ctLoiter:
		case ctLoiterUntil:
			// the mission command is loitering, process the loiter
			NavData.CTS = LoiterCalcCTS();
			break;

		case ctGotoWaypoint:
		case ctReturnToHome:
			// otherwise we are steering to waypoint
			// steering to waypoint, then set the course to steer to BTW
			// if outside the configured radius of the waypoint, then calculate the course to steer.
			// otherwise don't calculate course, that is hold course
			// this is to prevent dramatic and unnecessary course changes close to the waypoint.
			if (NavData.DTW >= Configuration.WPCourseHoldRadius) // || (NavData.DTW >= MissionValues.MissionList[StateValues.mission_index].boundary))
			{
				NavData.CTS = CalculateSailingCTS();
			}
			else
			{
				// we're inside the circle. hold course, but make sure its a sailable course
				NavData.CTS = LimitToSailingCourse(NavData.CTS);
				DecisionEventReason = DecisionEventReasonType::rApproachingWP;
				DecisionEvent = DecisionEventType::deHoldCourse;
			//	TelemetryLogging_Event_Decisions(CommandPort, Configuration.TelemetryLoggingMask);
				SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
			}
			break;

		case ctSteerWindCourse:
			// get current True Wind Angle (TWA) and compare with target Wind Angle
			// note: NavData.AWA has a 1 second latency. 
			NavData.CTS = wrap_360(NavData.HDG + (NavData.AWA - StateValues.SteerWindAngle));
			break;

		default:;
		}
		break;

	case vcsReturnToHome:
		// steering to waypoint, then set the course to steer to BTW
		NavData.CTS = CalculateSailingCTS();
		break;

	case vcsLoiter:
		NavData.CTS = LoiterCalcCTS();
		break;

	case vcsSteerMagneticCourse:
		// steering compass course, then set the course to steer to the command parameter angle.
		NavData.CTS = StateValues.SteerCompassBearing;
		break;

	case vcsSteerWindCourse:
		// get current True Wind Angle (TWA) and compare with target Wind Angle
		// note: NavData.AWA has a 1 second latency. 
		NavData.CTS = wrap_360(NavData.HDG + (NavData.AWA - StateValues.SteerWindAngle));
		break;

	case vcsFullManual:
	case vcsPartialManual:
	case vcsSetHome:
	case vcsResetMissionIndex:
		break;

	default:	;
	}
}

int CalculateSailingCTS(void)
{
	// V1.1 17/6/2019 
	// V1.2 11/4/2020 added CTE Correction to Direct to Waypoint Steering Course
	// V1.3 5/1/2021 updated to support Downwind tacking, hence four laylines.
	// V1.5 18/1/2021 added another criteria for assessing whether to sail the favoured tack. 
	//		That is, if we were sailing direct to waypoint, and now we are not, then go to favoured tack.
	//		Also, if we have just incremented a mission step, then go to favoured tack. (A new mission step is identified by the mission step duration being a few seconds..)  

	// every 5 seconds.
	// review how to get to Waypoint, and whether a tack is needed or not
	int SteeringCourse = -111; // dummy value

	// if BTW is sailable then return NavData.BTW with CTE correction;
	if (NavData.IsBTWSailable)
	{
		SteeringCourse = wrap_360(NavData.BTW - NavData.CTE_Correction); // subtract the offset (CTE Correction) to steer back to rhumb line to reduce CTE
		NavData.CourseType = SteeringCourseType::ctDirectToWayPoint;
	}
	else
	{
	  // not directly sailable to the Waypoint, hence need to tack up or tack down to waypoint.

	 //  if we were previously sailing direct to WP and now can't, then choose favoured tack
		// OR  if we have just incremented a mission step (in the last 8 seconds) then we need to re-assess and choose favoured tack
		//		we would get here within about 5 seconds, if it wasn't sailable, hence allow a bit more, i.e. 8 seconds.
		if ( (NavData.CourseType == SteeringCourseType::ctDirectToWayPoint ) ||
				((millis() - MissionValues.MissionCommandStartTime) < 8000) )
		{
			// given we were previously sailing direct to WP and now can't, then set the Tack to the Favoured tack
			// (Note: NavData.CourseType is set within the SetTack function)
			SteeringCourse = SetTack(NavData.FavouredTack, DecisionEventReasonType::rFavouredTack);
			NavData.TackDuration = 0;
		}

		switch(NavData.CourseType)
		{
		case SteeringCourseType::ctPortTack:
			// we are here because the BTW is not directly sailable.
			// hold the current tack until we reach the boundary
			// if we reach the starboard boundary then change to Starboard tack
			if (NavData.CTE > NavData.MaxCTE)
			{
				// swap to starboard tack Beating
				NavData.Manoeuvre = ManoeuvreType::mtGybe;
				NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;
				SteeringCourse = SetTack(SteeringCourseType::ctStarboardTack, DecisionEventReasonType::rPastBoundary);
				NavData.TackDuration = 0;
			} 
			else
			{
				// hold port tack Beating
				SteeringCourse = SetTack(SteeringCourseType::ctPortTack, DecisionEventReasonType::rNoChange);
			}
			break;

		case SteeringCourseType::ctStarboardTack:
			// if we reach the Port boundary then change to port tack.
			if (-NavData.CTE > NavData.MaxCTE)
			{
				// change to port tack Beating
				NavData.Manoeuvre = ManoeuvreType::mtGybe;
				NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;
				SteeringCourse = SetTack(SteeringCourseType::ctPortTack, DecisionEventReasonType::rPastBoundary);
				NavData.TackDuration = 0;
			}
			else
			{
				// hold starboard tack Beating
			 SteeringCourse = SetTack(SteeringCourseType::ctStarboardTack, DecisionEventReasonType::rNoChange);
			}
			break;


		case SteeringCourseType::ctPortTackRunning:
			// we are here because the BTW is not directly sailable.
			// hold the current tack until we reach the boundary
			// if we reach the PORT boundary then change to Starboard RUNNING tack
			if (-NavData.CTE > NavData.MaxCTE)
			{
				// swap to starboard tack Running
				//NavData.Manoeuvre = ManoeuvreType::mtGybe;
				//NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;
				SteeringCourse = SetTack(SteeringCourseType::ctStarboardTackRunning, DecisionEventReasonType::rPastBoundary);
				NavData.TackDuration = 0;
			}
			else
			{
				// hold port tack Running
				SteeringCourse = SetTack(SteeringCourseType::ctPortTackRunning, DecisionEventReasonType::rNoChange);
			}
			break;

		case SteeringCourseType::ctStarboardTackRunning:
			// if we reach the Starboard boundary then change to port RUNNING tack.
			if (NavData.CTE > NavData.MaxCTE)
			{
				// change to port tack Running
			//	NavData.Manoeuvre = ManoeuvreType::mtGybe;
			//	NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;
				SteeringCourse = SetTack(SteeringCourseType::ctPortTackRunning, DecisionEventReasonType::rPastBoundary);
				NavData.TackDuration = 0;
			}
			else
			{
				// hold starboard tack Running
				SteeringCourse = SetTack(SteeringCourseType::ctStarboardTackRunning, DecisionEventReasonType::rNoChange);
			}
			break;

		case SteeringCourseType::ctDirectToWayPoint:
			// we shouldn't get here.
			break;

		default:;
		}
	} // not IsBTWSailable

	return SteeringCourse;
}

int SetTack(SteeringCourseType Tack, DecisionEventReasonType Reason)
{
	// This sets the tack type and returns the steering course
	// also it logs an event
	// in coming Tack is Port or Starboard
	// V1.3 5/1/2021 updated to support Downwind tacking, hence four laylines.

	int SteeringCourse;
	NavData.CourseType = Tack;

	switch (Tack)
	{
	case SteeringCourseType::ctPortTack:
		DecisionEvent = DecisionEventType::deTackToPort;
		SteeringCourse = SteerCloseHauled(Tack);
		break;

	case SteeringCourseType::ctStarboardTack:
		DecisionEvent = DecisionEventType::deTackToStarboard;
		SteeringCourse = SteerCloseHauled(Tack);
		break;

	case SteeringCourseType::ctPortTackRunning:
		DecisionEvent = DecisionEventType::deTackToPortRunning;
		SteeringCourse = SteerDeepRunning(Tack);
		break;

	case SteeringCourseType::ctStarboardTackRunning:
		DecisionEvent = DecisionEventType::deTackToStarboardRunning;
		SteeringCourse = SteerDeepRunning(Tack);
		break;

	case SteeringCourseType::ctDirectToWayPoint:
		// we shouldn't get here.
		SteeringCourse = NavData.BTW;
		break;
	default:;
	}

	// log decision provide a reason specified.
	if (Reason != DecisionEventReasonType::rNoChange)
	{
		DecisionEventReason = Reason;
	///	TelemetryLogging_Event_Decisions(CommandPort, Configuration.TelemetryLoggingMask);
		SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
	}

	return SteeringCourse;
}

void UpdateTargetHeading(void)
{
	// called in the fast loop i.e. 50ms
	// Steering heading  or Helm Heading perhaps ?
	// apply a low pass filter to the TurnHDG to yield an instantaneous steering heading.
	// V1.1 15/2/2019 changed input from NavData.CTS to NavData.TurnHDG

	NavData.TargetHDG = TargetHeadingFilter.Filter(NavData.TurnHDG);
}

SteeringCourseType GetFavouredTack(NavigationDataType NavData)
{
	// Function to provide the favoured tack; i.e. closest to BTW.
	// use BTW and TWD
	// returns one of four tacks: Port or Starboard, beating or running
	// V1.3 5/1/2021 updated to support Downwind tacking, hence four laylines.

	SteeringCourseType FavouredTack;
	PointOfSailType pointofsailToWaypoint = GetPointOfSail(NavData.WindAngleToWaypoint);

	switch (pointofsailToWaypoint)
	{
	case PointOfSailType::psPortTackBeating:
		FavouredTack = SteeringCourseType::ctPortTack;
		break;

	case PointOfSailType::psPortTackRunning:
		FavouredTack = SteeringCourseType::ctPortTackRunning;
		break;

	case PointOfSailType::psStarboardTackBeating:
		FavouredTack = SteeringCourseType::ctStarboardTack;
		break;

	case PointOfSailType::psStarboardTackRunning:
		FavouredTack = SteeringCourseType::ctStarboardTackRunning;
		break;

	case PointOfSailType::psNotEstablished:
		FavouredTack = SteeringCourseType::ctNotEstablished;
		break;

	default:
		FavouredTack = SteeringCourseType::ctNotEstablished;
	}
	return FavouredTack;
}

int LimitToSailingCourse(int Course)
{
	// Restrict the course to a sailing course to guard against sailing too high.
	// V1.0 23/4/2019 John Semmens
	// V1.1 5/1/2021 added support for all 4 laylines, using PointOfSail to choose the quadrant.

	int SailingCourse;
	int AWA = AWA_Calculated(Course, NavData.TWD);
	int abs_AWA = abs(AWA);

	// look at whether the upwind course is Sailable, and also that the downwind course is sailable
	bool IsCourseSailable = (abs_AWA >= Configuration.MinimumAngleUpWind) && (abs_AWA <= (180 - Configuration.MinimumAngleDownWind));

	if (IsCourseSailable)
	{
		// if is Sailable then just sail the course
		SailingCourse = Course;
	}
	else
	{
		// Not sailable, therefore choose a layline.
		switch (NavData.PointOfSail)
		{
		case PointOfSailType::psPortTackBeating:
			SailingCourse = NavData.StarboardLayline;
			DecisionEvent = DecisionEventType::deTackToPort;
			break;

		case PointOfSailType::psStarboardTackBeating:
			SailingCourse = NavData.PortLayline;
			DecisionEvent = DecisionEventType::deTackToStarboard;
			break;

		case PointOfSailType::psPortTackRunning:
			SailingCourse = NavData.StarboardLaylineRunning;
			DecisionEvent = DecisionEventType::deTackToPortRunning;
			break;

		case PointOfSailType::psStarboardTackRunning:
			SailingCourse = NavData.PortLaylineRunning;
			DecisionEvent = DecisionEventType::deTackToStarboardRunning;
			break;

		default:
		 SailingCourse = Course;
		}

		DecisionEventReason = DecisionEventReasonType::rLimitToSailingCourse;
	///	TelemetryLogging_Event_Decisions(CommandPort, Configuration.TelemetryLoggingMask);
		SD_Logging_Event_Decisions(Configuration.SDCardLoggingMask);
	}
	return SailingCourse;
};

int SteerCloseHauled(SteeringCourseType Tack)
{
	// return a closehauled steering course for the given tack (Port or Starboard)
	// V1.0 5/5/2019 John Semmens.
	// V1.1 15/6/2019 Corrected sign reversal error for calculating port and starboard tacks

	int SteeringCourse = NavData.BTW; // default, not needed;

	switch (Tack)
	{
	case SteeringCourseType::ctPortTack:
		DecisionEvent = DecisionEventType::deTackToPort;
		SteeringCourse = wrap_360(NavData.HDG + (NavData.AWA + Configuration.MinimumAngleUpWind));
		break;

	case SteeringCourseType::ctStarboardTack:
		DecisionEvent = DecisionEventType::deTackToStarboard;
		SteeringCourse = wrap_360(NavData.HDG + (NavData.AWA - Configuration.MinimumAngleUpWind));
		break;

	case SteeringCourseType::ctDirectToWayPoint:
		// we shouldn't get here.
		SteeringCourse = NavData.BTW;
		break;
	default:;
	}
	return  SteeringCourse;
}

int SteerDeepRunning(SteeringCourseType Tack)
{
	// return a Deep Running steering course for the given tack (Port or Starboard, both Running and beating)
	// V1.0 2/1/2021 John Semmens.

	int SteeringCourse = NavData.BTW; // default, not needed;

	switch (Tack)
	{
	case SteeringCourseType::ctPortTack:
	case SteeringCourseType::ctPortTackRunning:
		DecisionEvent = DecisionEventType::deTackToPortRunning;
		SteeringCourse = wrap_360(NavData.HDG + (NavData.AWA - Configuration.MinimumAngleDownWind + 180));
		break;

	case SteeringCourseType::ctStarboardTack:
	case SteeringCourseType::ctStarboardTackRunning:
		DecisionEvent = DecisionEventType::deTackToStarboardRunning;
		SteeringCourse = wrap_360(NavData.HDG + (NavData.AWA + Configuration.MinimumAngleDownWind - 180));
		break;

	case SteeringCourseType::ctDirectToWayPoint:
		// we shouldn't get here.
		SteeringCourse = NavData.BTW;
		break;
	default:;
	}
	return  SteeringCourse;
}


PointOfSailType GetPointOfSail(int WindAngle)
{
	// function to return a point of Sail based on a Wind Angle.
	// Wind Angle is the angle off the bow. Negative Wind Angle refers to Port Tack. Poistive Starboard Tack.
	// it is expected that Wind Angle paramter pased in is a wrapped to +/- 180 degrees.
	// Called in 5s loop and used to set NavData.CourseType
	// V1.0 21/9/2016 John Semmens.
	// V1.2 10/2/2019 correct a reversal error of port vs starboard
	// V1.3 1/2/2021 changed to divide the possible courses into four quadrants.

	PointOfSailType PointOfSail = PointOfSailType::psNotEstablished;

	// Port Tack - negative wind angle
	if (WindAngle <= 0)
	{
		WindAngle = -WindAngle; // reverse the sign to make it positive

		// Port Tack - 0 to 90 degrees
		if (WindAngle <= 90)
			PointOfSail = PointOfSailType::psPortTackBeating;

		//port tack - 90 to 180 degrees
		if (WindAngle > 90)
			PointOfSail = PointOfSailType::psPortTackRunning;
	}
	else // Starboard Tack - positive wind angle
	{
		// Starboard Tack - 0 to 90 degrees
		if (WindAngle <= 90)
			PointOfSail = PointOfSailType::psStarboardTackBeating;

		//Starboard tack - 90 to 180 degrees
		if (WindAngle > 90)
			PointOfSail = PointOfSailType::psStarboardTackRunning;
	}
	return PointOfSail;
}

bool IsBTWSailable(NavigationDataType NavData)
{
	// function to determine if the BTW is sailable or not.
	// that is, can the course be laid.
	// This is determined by comparing Wind Angles off the bow, against sailing limits specified in the configuration
	// This only approximate, because we only have apparent wind angle, not True Wind Angle.
	// This is due to not having a measure of wind speed to establish the True Wind.
	// V1.0 19/9/2016 John Semmens
	// V1.1 11/02/2019 removed test for whether the downwind course is sailable.
	// V1.2 13/6/2019 added SailableAngleMargin to the MinimumAngleUpWind to inhibit tacking a bit.
	// V1.3 5/1/2021 reinstate downwind angle test

	// get current heading from IMU compared to CTS to find the expected change in direction.
	// then apply that to the current Wind Angle to fnd the expected wind angle.
	// then compare the expected Wind angle with the wind Angle Sailing limits.
	int abs_WindAngleToWaypoint = abs(NavData.WindAngleToWaypoint);

	//add a sailing angle margin before declaring that it is sailable. this to inhibit tacking when the angle is marginal. i.e. ensure angle is not marginal.
	// this is like hysteresis.
	int SailableAngleMargin = Configuration.SailableAngleMargin;

	// if outside of the boundary then reduce the SailableAngleMargin to half to encourage a tack to come back
	if (abs(NavData.CTE) > NavData.MaxCTE)
	{
		SailableAngleMargin = SailableAngleMargin * 0.5;
	}

	bool IsCourseSailable = (abs_WindAngleToWaypoint >= (Configuration.MinimumAngleUpWind + SailableAngleMargin)) 
			&& (abs_WindAngleToWaypoint <= (180 - (Configuration.MinimumAngleDownWind))); // + SailableAngleMargin)));

	return IsCourseSailable;
}