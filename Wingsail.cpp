// 
// 
// 

#include "Wingsail.h"
#include "configValues.h"
#include "CommandState_Processor.h"
#include "RadioControl.h"
#include "Navigation.h"
#include "Loiter.h"
#include "GPS.h"
#include "BluetoothConnection.h"
#include "MPU9250.h"
#include "WindAngle_2950.h"
#include "WearTracking.h"
#include "Mission.h"

extern HardwareSerial *Serials[];
extern BTStateType BTState;

extern configValuesType Configuration;		// stucture holding Configuration values; preset variables
extern WingSailType WingSail;
extern StateValuesStruct StateValues;
extern NavigationDataType NavData;
extern LoiterStruct LoiterData;
extern RC_IN_Type RC_IN;
extern MPU9250 WingSailAngleSensor;		// The IMU including Magnetic Compass and Gyro 
extern WearCounter TrimTabUsage;
extern MissionValuesStruct MissionValues;

const uint16_t Deadband = 10; // us

void wingsail_init(void)
{
	// set to forward, at least until we have control over the state. 
	WingSail.State = wsForward;
	wingsail_update();
};

void wingsail_update(void)
{
	// called every 5 seconds normally or 1 second if in manual mode.
	// also called in 50ms loop to track changes rapidly.

	//static uint16_t prev_SailIn_us;
	uint16_t SailIn_us;

	// sailing mode 
	// idle/forward/manual
	switch(StateValues.CommandState)
	{
	case vcsFollowMission:
		// We're following mission and not steering a wind course as a mission step
		if (MissionValues.MissionList[StateValues.mission_index].cmd != MissionCommandType::ctSteerWindCourse)
		{
			if (GPS_LocationIs_Valid(NavData.Currentloc) && NavData.next_WP_valid)
			{
				// set sail to forward sailing mode
				WingSail.State = wsForward;
				AutoSetWingSail(WingSail.State);
			}
			else
			{
				// set sail to idle
				WingSail.State = wsIdle;
				AutoSetWingSail(WingSail.State);
			}
		}
		else // We are following mission and we are steering a wind course as a mission step
		{
			// explicitly set the TrimTab Angle
			SetTrimTabAngle(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
		}

		break;

	case vcsLoiter:
		if (GPS_LocationIs_Valid(NavData.Currentloc) &&
			(LoiterData.LoiterState != LoiterStateType::lsNotLoitering)
			)
		{
			// set sail to forward sailing mode
			WingSail.State = wsForward;
			AutoSetWingSail(WingSail.State);
		}
		else
		{
			// set sail to idle
			WingSail.State = wsIdle;
			AutoSetWingSail(WingSail.State);
		}
		break;

	case vcsReturnToHome:
		if (GPS_LocationIs_Valid(NavData.Currentloc) && StateValues.home_is_set)
		{
			// set sail to forward sailing mode
			WingSail.State = wsForward;
			AutoSetWingSail(WingSail.State);
		}
		else
		{
			// set sail to idle
			WingSail.State = wsIdle;
			AutoSetWingSail(WingSail.State);
		}
		break;

	case vcsPartialManual:
	case vcsSetHome:
	case vcsResetMissionIndex:
	case vcsSteerMagneticCourse:
	case vcsSteerWindCourse:
		WingSail.State = wsForward;
		AutoSetWingSail(WingSail.State);
		break;

	case vcsFullManual:
		// pass through the RC Input channels to the BT Serial Servo
		SailIn_us = RC_IN.Channel[Configuration.RC_IN_Channel_Sail];
		WingSailServo(SailIn_us);
		break;

	case vcsIdle:
		WingSail.State = wsIdle;
		AutoSetWingSail(WingSail.State);
		break;

	default:;
		WingSail.State = wsIdle;
		AutoSetWingSail(WingSail.State);
	};
};

void AutoSetWingSail(WingSailStateType WingSailState)
{
	// called every 5 seconds normally or 1 second if in manual mode.
	// set the trim tab in accordance with current conditions, and current state
	WingSail.TrimTabAngle = CalcTrimTabAngle(WingSail.Angle, WingSailState);
	SetTrimTabAngle(WingSail.TrimTabAngle);
}

void SetTrimTabAngle(int TrimTabAngle)
{
	// set the trim tab angle 
	WingSail.Servo_microseconds = TrimTabAngle_to_us(TrimTabAngle);
	WingSailServo(WingSail.Servo_microseconds);
}



int CalcTrimTabAngle(int WingSailAngle, WingSailStateType WingSailState)
{
	// return the required angle to apply to the TrimTab to drive the vessel in accordance with
	// wingsail state, and the current wind angle.
	// called every 5 seconds normally or 1 second if in manual mode.
	// V1.1 update to add Config TrimTabDefaultAngle

	IdentifyCurrentTack(WingSailAngle);

	int TrimTabAngle = 0;

	switch (WingSailState) {
	case WingSailStateType::wsIdle:
		TrimTabAngle = 0;
		break;

	case WingSailStateType::wsForward:
		switch (WingSail.Tack) {
		case WingSailTackType::wsPortTack:
			TrimTabAngle = -Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsStarboardTack:
			TrimTabAngle = +Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsHeadToWind:
		default:
			TrimTabAngle = 0;
		};
		break;

	case WingSailStateType::wsReverse:
		switch (WingSail.Tack) {
		case WingSailTackType::wsPortTack:
			TrimTabAngle = +Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsStarboardTack:
			TrimTabAngle = -Configuration.TrimTabDefaultAngle;
			break;
		case WingSailTackType::wsHeadToWind:
		default:
			TrimTabAngle = 0;
		};
		break;

	default:
		TrimTabAngle = 0;
	}
	return TrimTabAngle;
}


int TrimTabAngle_to_us(int TrimTabAngle)
{
	// convert requested trim tab angle to the corresponding Servo Signal in microseconds.
	int ServoMicroSeconds = (Configuration.TrimTabScale * TrimTabAngle) + Configuration.TrimTabOffset;

	return ServoMicroSeconds;
}

void WingSailServo(int SailIn_us)
{
	// send the serial message to the Bluetooth Servo, but only if connected.
	static uint16_t prev_SailIn_us;

	if ((abs(SailIn_us - prev_SailIn_us) >= Deadband) && (BTState == BTStateType::Connected))
	{
		prev_SailIn_us = SailIn_us;

		// send the serial message for the servo 
		//WingSailServo(SailIn_us);

		// send the serial message for the servo 
		(*Serials[Configuration.BluetoothPort]).print("srv,");
		(*Serials[Configuration.BluetoothPort]).println(SailIn_us);
		TrimTabUsage.TrackServoUsage(SailIn_us);
	}
};

void IdentifyCurrentTack(int WingSailAngle)
{
	// calculate whether the sail is on port tack or starboard tack based on Wing Angle.
	// This is useful for identifying transitions of the wingsail through manoeuvres.

	// Wingsail Angle varies from -180 to 0 to +180

	// V1.0 5/2/2018 John Semmens.
	const int HeadToWindAngle = 3; // was 5 now 3 degrees 25/6/2020

	// port tack is more than x degrees off
	if (WingSailAngle <= -HeadToWindAngle)
	{
		WingSail.Tack = wsPortTack;
	}

	// starboard tack is more than x degrees off
	if (WingSailAngle >= +HeadToWindAngle)
	{
		WingSail.Tack = wsStarboardTack;
	}

	//  Head to Wind is within x degrees of zero.
	if (WingSailAngle > -HeadToWindAngle && WingSailAngle < HeadToWindAngle)
	{
		WingSail.Tack = wsHeadToWind;
	}
};

void Wingsail_TrackTackChange(void)
{
	// called in the fast loop 50ms
	
	static WingSailTackType WingSailTackPrev;

	IdentifyCurrentTack(WingSail.Angle); // convert WingSail Angle to a WingSail.Tack state

	// if there's a change of Wingsail Tack Status, then send an update to the wingsail
	if (WingSailTackPrev != WingSail.Tack)
	{
		WingSailTackPrev = WingSail.Tack;
		wingsail_update();
	}
};