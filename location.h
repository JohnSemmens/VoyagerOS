// location.h

#ifndef _LOCATION_h
#define _LOCATION_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// return distance in meters between two locations
float get_distance(const struct Location &loc1, const struct Location &loc2);

// return bearing in centi-degrees between two locations
//int32_t get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

// return bearing in degrees between two locations
int get_bearing(const struct Location &loc1, const struct Location &loc2);
float get_bearingf(const struct Location& loc1, const struct Location& loc2);

// wrap an angle in degrees to -180..180
int wrap_180(int angle);
float wrap_360(float angle);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool location_passed_point(const struct Location &location,
	const struct Location &point1,
	const struct Location &point2);

bool reached_waypoint_point(const struct Location &location,
	const struct Location &point1,
	const struct Location &point2,
	int WP_Boundary);

// return the Cross Track Error from a line from point1 to point2 and the position at location
int get_CTE(const struct Location& location, const struct Location& point1, const struct Location& point2);

#endif