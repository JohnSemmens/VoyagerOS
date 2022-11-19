#include "location.h"
#include "sim_vessel.h"
#include "AP_Math.h"
#include "configValues.h"
#include "sim_weather.h"

extern configValuesType Configuration;		// stucture holding Configuration values; preset variables
extern double SteeringServoOutput;
extern sim_weather simulated_weather;

void sim_vessel::update()
{
	// update the simulated vessel postion and attitude 
	
	// maintain an elapsed time between updates.
	unsigned long current_time = millis();
	update_time_ms = current_time - prev_update_time;
	prev_update_time = current_time;

	if (update_time_ms < 10000) // < 10seconds
	{
		// calculate the simulated distance moved based on the VPP and elapsed time since last update
		SOG_mps = vpp(  wrap_180(simulated_weather.WindDirection - Heading), simulated_weather.WindSpeed);
		float UpdateDistance = SOG_mps * update_time_ms / 1000;

		// calculate the simulated heading change based on rudder position, and SOG (sort of boat speed) and elapsed time since last update
		float TurnRateFactor = 40; // 30; // degrees/second
		HeadingChange = ((SteeringServoOutput - Configuration.pidCentre) / 500.0) * SOG_mps / 1.0 * (update_time_ms / 1000.0) * TurnRateFactor; // update heading, in degrees
		Heading = wrap_360(Heading - HeadingChange);

		// update the simulated vessel postion  based on new heading and distance.
		location_update(Currentloc, (float)Heading, UpdateDistance);

		if ((simulated_weather.WindDirection - Heading) > 0)
		{
			WingsailAngle = wrap_180(simulated_weather.WindDirection - Heading );
		}
		else
		{
			WingsailAngle = wrap_180(simulated_weather.WindDirection - Heading );
		}
		
	}
}

float sim_vessel::vpp(int windAngle, int windspeed) // degrees from head-to--wind, knots
{
	// V1.0 9/4/2021 initial simple VPP program
	
	int AbsWindAngle = abs(windAngle);
	float velocity; // Metres/second

	// 10 knot wind at 90 degrees yields 1 m/s boat speed.

	if (AbsWindAngle <= 90) // 0 to 90 degrees, increase speed proportionally with angle
	{
		velocity = AbsWindAngle / 90.0 * windspeed / 10.0;
	}
	else
	{
		velocity =  windspeed / 10.0; // speed is constant in lower half of polar diagram.
	}

	return velocity;
}