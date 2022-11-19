// Waypoints.h

#ifndef _WAYPOINTS_h
#define _WAYPOINTS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between vesel and next waypoint.  Meters
//static float wp_distance;

// Distance between previous and next waypoint.  Meters
//static int32_t wp_totalDistance;

struct  Location {
	int32_t lat;        // Latitude * 10**7  
	int32_t lng;        // Longitude * 10**7
};

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
//// The home location used for RTL.  The location is set when we first get stable GPS lock
//static struct   Location home;
//
//// Flag for if we have gps lock and have set the home location
//static bool     home_is_set;

//// The location of the previous waypoint.  Used for track following and altitude ramp calculations
//static struct   Location prev_WP;
//
//// The location of the current/active waypoint.  Used for track following
//static struct   Location next_WP;

// The location of the active waypoint in Guided mode.
//static struct   Location guided_WP;






#endif

