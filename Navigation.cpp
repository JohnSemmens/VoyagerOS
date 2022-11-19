// This considers the Next Waypoint and the Previous Waypoint and the permitted Cross Track Error (Boundary).
// It then calulates the desired Distance and Bearing to Waypoint.
// It also considers whether the current leg of the mission has been completed.

// V1.1 30/8/2016 updated for reorganisation of navigation global variables into a structure
// V1.2 9/11/2016 Corrected Apparent Wind Angle to Waypoint to use true Heading
// V1.3 11/7/2019 added low pass filter for roll angle
// V1.4 11/4/2020 added CTE_Correction, and added improved CTE calculation: get_CTE()
// V1.6 20/6/2020 added VMG and VMC, SOG_Avg and COG_Avg
// V1.7 2/1/2021 added Laylines for running to the NavData structure
//				 added Port and Starboard Tack Running to SteeringCourseType enum

#include "location.h"
#include "Waypoints.h"
#include "Navigation.h"
#include "USFS_imu.h"
#include "configValues.h"
#include "GPS.h"
#include "SailingNavigation.h"
#include "AP_Math.h"
#include "Wingsail.h"
#include "WindAngle_2950.h"
#include "MPU9250.h"
#include "Filters.h"
#include "sim_vessel.h"

extern IMUStruct myIMU;
extern NavigationDataType NavData;
extern configValuesType Configuration;
extern WingSailType WingSail;
extern MPU9250 WingSailAngleSensor;
extern StateValuesStruct StateValues;
extern bool UseSimulatedVessel;
extern sim_vessel simulated_vessel;

LowPassAngleFilter TrueWindFilter;
LowPassFilter RollFilter;
LowPassFilter HeadingErrorFilter;
LowPassFilter SOGFilter;
LowPassAngleFilter COGFilter;

void NavigationUpdate_SlowData(void)
{
	// calculate the DTW, BTW and CTE for the current Previous and Next Waypoints,
	// using the current location.
	// This is expected to be called from a slow loop because the values calculated here don't change rapidly
	// this is called about every 5 seconds
	// V1.0 4/8/2016 John Semmens
	// V1.1 6/8/2016 updated to only perform calculations if the next Waypoint is valid
	// V1.2 9/11/2016 Corrected Apparent Wind Angle to Waypoint to use true Heading
	//					Add handling for case when the WP  Valid flags are not valid.
	// V1.4 12/11/2017 updated to change BRL to RLB.
	// V1.5 21/10/2018 updated to change test for past waypoint to require it to be within range.

	if (NavData.next_WP_valid && GPS_LocationIs_Valid(NavData.Currentloc)) {
		NavData.RLB = get_bearing(NavData.prev_WP, NavData.next_WP);
		NavData.BTW = get_bearing(NavData.Currentloc, NavData.next_WP);
		NavData.CDA = wrap_180(NavData.RLB - NavData.BTW); // Course deviation Angle // angle between the rumb line course between waypoints, and the bearing to waypoint
		NavData.DTW = get_distance(NavData.Currentloc, NavData.next_WP);
		NavData.CTE = get_CTE(NavData.Currentloc, NavData.prev_WP, NavData.next_WP);  // New improved Cross Track Error calc. +ve Starboard side of course.
		NavData.CTE_Correction = get_CTE_Correction(NavData); // this is a steering correction based on CTE
		NavData.PastWP = reached_waypoint_point(NavData.Currentloc, NavData.prev_WP, NavData.next_WP, NavData.MaxCTE);	

		NavData.WindAngleToWaypoint = wrap_180(NavData.TWD - NavData.BTW); // WindAngleToWaypoint is difference between TWD and BTW.
		NavData.IsBTWSailable = IsBTWSailable(NavData);
		NavData.FavouredTack = GetFavouredTack(NavData);

		NavData.VMG = NavData.SOG_Avg * cos(wrap_180(NavData.COG_Avg - NavData.TWD) / 57.2957795f);
		NavData.VMC = NavData.SOG_Avg * cos(wrap_180(NavData.COG_Avg - NavData.BTW) / 57.2957795f);

		if (StateValues.home_is_set)
		{
			NavData.DTH = get_distance(NavData.Currentloc, StateValues.home);
			NavData.BTH = get_bearing(NavData.Currentloc, StateValues.home);
		}
		else
		{
			NavData.DTH = 0;
			NavData.BTH = 0;
		}

	}
	else {
		NavData.RLB = 0;
		NavData.BTW = 0;
		NavData.CDA = 0;
		NavData.DTW = 0;
		NavData.CTE = 0;
		NavData.CTE_Correction = 0;
		NavData.PastWP = false;
		NavData.WindAngleToWaypoint = 0;
		NavData.IsBTWSailable = false;
		NavData.VMG = 0;
		NavData.VMC = 0;

		NavData.DTH = 0;
		NavData.BTH = 0;
	}

	NavData.PointOfSail = GetPointOfSail(NavData.AWA);
	// calculate the bearing lines of the both tacks for Beating 
	NavData.PortLayline = wrap_360(NavData.TWD - Configuration.MinimumAngleUpWind);
	NavData.StarboardLayline = wrap_360(NavData.TWD + Configuration.MinimumAngleUpWind);

	// calculate the bearing lines of the both tacks for Running
	NavData.PortLaylineRunning = wrap_360(NavData.TWD + 180 + Configuration.MinimumAngleDownWind);
	NavData.StarboardLaylineRunning = wrap_360(NavData.TWD + 180 - Configuration.MinimumAngleDownWind);
}

void NavigationUpdate_MediumData(void)
{
	// This is expected to be called from a one second loop .
	// V1.0 11/7/2019 John Semmens

	NavData.ROLL_Avg = RollFilter.Filter(myIMU.roll);
	NavData.SOG_Avg = SOGFilter.Filter(NavData.SOG_mps);
	NavData.COG_Avg = COGFilter.Filter(NavData.COG);
}

void NavigationUpdate_FastData(void)
{
	// Calculate True Heading From Magnetic Heading.
	// This is expected to be called from a fast loop .
	// about 50ms
	// V1.0 31/10/2016 John Semmens
    // V1.1	6/4/2019 added CompassOffsetAngle for handling mounting orientation
	// V1.2 11/7/2019 added Heading Error calculation and correction.

	NavData.HDG_Mag = int(wrap_360(myIMU.heading + Configuration.MagnetVariation + Configuration.CompassOffsetAngle));


	// calculate and error value from the GPS and dampen it using low pass filter
	int RawHeadingError;
	if (NavData.SOG_mps > 0.2) // if the SOG is reasonable then assume the 
	{
		RawHeadingError = wrap_180(NavData.HDG_Mag - NavData.COG);
	}
	else
	{
		RawHeadingError = 0;
	}
	
	// apply a low pass filter
	NavData.HDG_Err = HeadingErrorFilter.Filter(RawHeadingError);

	// Get Vessel Heading; either real or simulated.
	if (!UseSimulatedVessel)
	{
		NavData.HDG = int(wrap_360(NavData.HDG_Mag - NavData.HDG_Err));
	}
	else
	{
		NavData.HDG = simulated_vessel.Heading;
	}

}

void GetApparentWind(void)
{
	// use the  wingsail angle to deduce AWA.
	// Add a small amount to the Wingsail angle related to the current Trimtab setting to offset the result.
	// V1.0 21/4/2019
	// V1.1 11/4/201 add simulation support. 

	const float AWATrimTabFactor = 0.2;

	// Get Vessel AWA; either real or simulated.
	if (!UseSimulatedVessel)
	{
		NavData.AWA = WingSailAngleSensor.MagneticAngle + (WingSail.TrimTabAngle * AWATrimTabFactor);
	}
	else
	{
		NavData.AWA = simulated_vessel.WingsailAngle + (WingSail.TrimTabAngle * AWATrimTabFactor);
	}
};

void Navigation_Init(void)
{
	// initialize the Lowpass Filters 
	// This should be called at start up.
	TrueWindFilter.FilterConstant = 0.1;

	RollFilter.FilterConstant = 0.1;
	SOGFilter.FilterConstant = 0.1;
	COGFilter.FilterConstant = 0.1;

	HeadingErrorFilter.FilterConstant = 0.005;
}

void GetTrueWind(void)
{
	// calculate the true wind 
	// V1.0 26/5/2018 John Semmens.
	// V1.1 6/5/2019
	// V1.2 24/10/2020 added AWD as an uncompensated (cleaner) version of TWD usefule in detecting a Gybe motion.
	// V1.3 10/1/2021 adjust TWD so the offset tapers off to zero as the AWA approaches 180 degrees (wind from behind).
	// V1.4 11/1/2021 corrected and verified - use "lcd,2" to test.

	// called in the one second loop.

	// this is a hack approximation of TWD.

	//int TWD_Offset=0;

	if (NavData.AWA > 0)
		// stbd tack
		NavData.TWD_Offset = Configuration.TWD_Offset * (180.0 - NavData.AWA)/180.0;  // +ve offset
	else 
		// port tack
		NavData.TWD_Offset = Configuration.TWD_Offset * -(180.0 + NavData.AWA)/180.0;  // -ve offset

	NavData.AWD = TrueWindFilter.Filter(wrap_360(NavData.HDG + NavData.AWA));
	NavData.TWD = wrap_360(NavData.AWD + NavData.TWD_Offset);
}


void UpdateTurnHeadingV2(void)
{
	// called in the 1 second loop.
	// check if the CTS has changed and caused a change in sign of the AWA. (i.e do we need to tack).
	// V2.0 15/5/2019 John Semmens
	// V2.1 30/5/2019 reworking the state-machine - debugging
	// V2.2 11/6/2019 widened running tolerance from 10 to 30 degrees
	// V2.3 24/10/2020 change the target heading during a gybe to based on AWD rather than TWD. 
	//			This is because the TWD is poor approximation, which corrupts the wind direction, especially when running, and may add to instability.

	static int Prev_CTS;

	// get the Current and Previous AWA based on TWD and Current and Previous CTS.
	int CurrentAWA = AWA_Calculated(NavData.CTS, NavData.TWD);
	int PreviousAWA = AWA_Calculated(Prev_CTS, NavData.TWD);

	// if there's a change of sign of the calculated AWA  AND  the mode is to Gybe
	// then COMMENCE
	if ( ((CurrentAWA * PreviousAWA) < 0) && (Configuration.TackingMethod == ManoeuvreType::mtGybe) )
	{
		NavData.ManoeuvreState = ManoeuvreStateType::mstCommence;
	}

	switch (NavData.ManoeuvreState)
	{
	case ManoeuvreStateType::mstCommence:
		// set the TurnHDG to dead downwind,
		NavData.TurnHDG = wrap_360(NavData.AWD + 180);

		// check if we are running yet. check that we're within 30 degrees of square running
		if (abs(wrap_180(NavData.AWA + 180)) < 30)
		{
			NavData.ManoeuvreState = ManoeuvreStateType::mstRunning;
		}
		break;

	case ManoeuvreStateType::mstRunning:
		// once change state to complete
		NavData.ManoeuvreState = ManoeuvreStateType::mstComplete;
		break;

	case ManoeuvreStateType::mstComplete:
	case ManoeuvreStateType::mstNone:
		NavData.TurnHDG = NavData.CTS;
		break;

	default:
		NavData.TurnHDG = NavData.CTS;
	}

	Prev_CTS = NavData.CTS;
}

int get_CTE_Correction(NavigationDataType NavData)
{
	// function to return a correction angle for the Cross Track Error (CTE)
	// The Correction has a positve value on the Starboard side of the course (the same as CTE).
	// Hence, the correction needs to be subracted from the calculated course.
	// The correction angle has a gain adjustment as one of the stored parameters: CTE_CorrectionGain
	// The correction is applied as a proportion of CTE versus Max CTE.
	// The correction is capped at a nominal maximum angle. 
	// This absolute maximum correction should be 90 degrees, or else it could start going backwards.
	// We'll cap it at lower values however.
	// 11/4/2020

	const int Max_Correction = 45; // degrees
	int CTE_Correction = 0;
	
	if (NavData.MaxCTE != 0) // guard against divide by zero
	{
		CTE_Correction = (float(NavData.CTE) / float(NavData.MaxCTE)) * Configuration.CTE_CorrectionGain;
	}

	// cap the correction angle to a nominated max. -- positive side
	if (CTE_Correction >= Max_Correction)
	{
		CTE_Correction = Max_Correction;
	}

	// cap for the negative side.
	if (CTE_Correction <= -Max_Correction)
	{
		CTE_Correction = -Max_Correction;
	}

	return CTE_Correction;
}


int AWA_Calculated(int CTS, int TWD)
{
	// function to return an approximate AWA based on the CTS and True Wind.
	// positive values are Starboard Tack
	// negative values are Port Tack
	// V1.0 22/4/2019 John Semmens

	return wrap_180(TWD - CTS);
}