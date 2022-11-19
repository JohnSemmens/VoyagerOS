#pragma once

#include "waypoints.h"

class sim_vessel
{
private:
	unsigned long prev_update_time;

public:
	int Heading; // degrees
	int Pitch; 
	int Roll; 

	Location Currentloc;

	float SOG_mps;		 // Speed Over Ground -- metres/second

	int WingsailAngle;

	void update();
	unsigned long update_time_ms;
	int HeadingChange;

	float vpp(int windAngle, int windspeed); // degrees from head-to--wind, knots
};

