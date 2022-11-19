// Handle the low level Steering
// use a PID to control the Steering servo(s) based on a target heading error.
// only operate the operate the automated steering if location is valid 
// or use the RC steering channel if in a manual type mode.

// V1.0 13/11/2016 
// V1.1 25/4/2018 updated 

#include "Steering.h"
#include "CommandState_Processor.h"
#include "configValues.h"
#include "RadioControl.h"
#include "location.h"
#include "Navigation.h"
#include "PID_v1.h"
#include "GPS.h"
#include "USFS_IMU.h"
#include "Loiter.h"
#include "WearTracking.h"

extern StateValuesStruct StateValues;
extern configValuesType Configuration;
extern IMUStruct myIMU;
extern NavigationDataType NavData;
extern RC_IN_Type RC_IN;
extern double SteeringServoOutput;
extern LoiterStruct LoiterData;
extern WearCounter PortRudderUsage;
extern WearCounter StarboardRudderUsage;

// Declare the Steeering PID 
double pidTargetHdgError = 0;
double pidActualHdgError, pidServoOutput;
PID SteeringPID(&pidActualHdgError, &pidServoOutput, &pidTargetHdgError, 5, 0, 0, DIRECT);

void SteeringFastUpdate(void)
{
	// Called from the Fast Loop.
	// 
	// This procedures operates the rudder.
	// It supports a single rudder or dual rudder.
	// The source of Rudder control is based on current CommandState.
	// For manual control type modes, use the RC input steering channel.
	// For automatically controlled modes, use the output of the Steering PID.
	// Calculate a heading error and then operate the rudder servo via the steering PID.
	// V1.0 14/11/2016 John Semmens
	// V1.2 24/4/2018 added offset to PID output by adding value: Configuration.pidCentreOffset
	// V1.3 28/4/2018 added criteria to loitering to only steer while on the approach or inbound phases.
	// V1.4 18/2/2019 added wear tracking for the Servos

	switch (StateValues.CommandState)
	{
	case vcsFollowMission:
		if (GPS_LocationIs_Valid(NavData.Currentloc) && NavData.next_WP_valid) 
		{
			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsLoiter:
		if ( GPS_LocationIs_Valid(NavData.Currentloc) &&
			(LoiterData.LoiterState != LoiterStateType::lsNotLoitering)
			)
		{
			// only steer if we are loitering.

			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsReturnToHome:
		if (GPS_LocationIs_Valid(NavData.Currentloc) && StateValues.home_is_set) 
		{
			// calculate the heading error  -180 to +180
			pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

			SteeringPID.Compute();
			SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		}
		break;

	case vcsSteerMagneticCourse:
	case vcsSteerWindCourse:
		pidActualHdgError = (double)wrap_180(NavData.HDG - NavData.TargetHDG);

		SteeringPID.Compute();
		SteeringServoOutput = pidServoOutput + Configuration.pidCentre;
		break;

	case vcsFullManual:
	case vcsPartialManual:
		// pass through the RC Input channels to the Servo Ouput Channels to allow manaual steering and motor control
		SteeringServoOutput = RC_IN.Channel[Configuration.RC_IN_Channel_Steering];
		break;

	case vcsSetHome:
	case vcsResetMissionIndex:
		break;
	default:	;
	}

	if (Configuration.DualRudder)
	{
		if (NavData.ROLL_Avg > 5) 
		{ // operate the Starboard Rudder
			Servo_Out(Configuration.Servo_Channel_Steering_Stbd, SteeringServoOutput);
			StarboardRudderUsage.TrackServoUsage(SteeringServoOutput);
		}
		else
		{ // operate the  Port Rudder
			Servo_Out(Configuration.Servo_Channel_Steering, SteeringServoOutput);
			PortRudderUsage.TrackServoUsage(SteeringServoOutput);
		}
	}
	else
	{ // operate a single Rudder
		Servo_Out(Configuration.Servo_Channel_Steering, SteeringServoOutput);
		PortRudderUsage.TrackServoUsage(SteeringServoOutput);
	}
}

void SteeringPID_Init(void)
{
	// initial the Steering PID using config values and start it.
	// V1.0 John Semmens
	// V1.1 25/4/2018 added the Centre value. This required when using PID set to P only (with no ID)

	SteeringPID.SetControllerDirection(Configuration.pidDirection);
	SteeringPID.SetTunings(Configuration.pidKp, Configuration.pidKi, Configuration.pidKd);
	SteeringPID.SetOutputLimits(Configuration.pidOutputmin, Configuration.pidOutputmax);
	SteeringPID.SetMode(AUTOMATIC); // switch on the PID
}

