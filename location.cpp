/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * location.cpp
 * Copyright (C) Andrew Tridgell 2011
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  this module deals with calculations involving struct Location
 */

#include "location.h"
#include <stdlib.h>
#include "AP_Math.h"
#include "Waypoints.h"

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

float longitude_scale(const struct Location &loc)
{
    static int32_t last_lat;
    static float scale = 1.0;
    if (fabs(last_lat - loc.lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    scale = constrain_float(scale, 0.01f, 1.0f);
    last_lat = loc.lat;
    return scale;
}



// return distance in meters between two locations
float get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
    return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}

// return distance in centimeters to between two locations
uint32_t get_distance_cm(const struct Location &loc1, const struct Location &loc2)
{
    return get_distance(loc1, loc2) * 100;
}


// return bearing in degrees between two locations
int get_bearing(const struct Location &loc1, const struct Location &loc2)
{
	int32_t off_x = loc2.lng - loc1.lng;
	int32_t off_y = (loc2.lat - loc1.lat) / longitude_scale(loc2);
	int bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;
	if (bearing < 0) bearing += 360;
	return bearing;
}

// return bearing as float in degrees between two locations
float get_bearingf(const struct Location& loc1, const struct Location& loc2)
{
    int32_t off_x = loc2.lng - loc1.lng;
    int32_t off_y = (loc2.lat - loc1.lat) / longitude_scale(loc2);
    float bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;
    if (bearing < 0) bearing += 360;
    return bearing;
}



// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool location_passed_point(const struct Location &location,
                           const struct Location &point1,
                           const struct Location &point2)
{
    // the 3 points form a triangle. If the angle between lines
    // point1->point2 and location->point2 is greater than 90
    // degrees then we have passed the waypoint
    Vector2f loc1(location.lat, location.lng);
    Vector2f pt1(point1.lat, point1.lng);
    Vector2f pt2(point2.lat, point2.lng);
    float angle = (loc1 - pt2).angle(pt1 - pt2);
    if (isinf(angle)) {
        // two of the points are co-located.
        // If location is equal to point2 then say we have passed the
        // waypoint, otherwise say we haven't
        if (get_distance(location, point2) == 0) {
            return true;
        }
        return false;
    } else if (angle == 0) {
        // if we are exactly on the line between point1 and
        // point2 then we are past the waypoint if the
        // distance from location to point1 is greater then
        // the distance from point2 to point1
        return get_distance(location, point1) >
               get_distance(point2, point1);

    }
    if (degrees(angle) > 90) {
        return true;
    }
    return false;
}

// return the Cross Track Error from a line defined by point1 to point2, and the position at location
int get_CTE(const struct Location& location, const struct Location& point1, const struct Location& point2)
{
    float RLB = get_bearingf(point1, point2);
    float BTW = get_bearingf(location, point2);
    float CDA = RLB - BTW;
    float DTW = get_distance(location, point2);

    return int(sin(CDA / 57.2957795f)* DTW);
};


/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(radians(bearing))*distance;
    float ofs_east  = sinf(radians(bearing))*distance;
    location_offset(loc, ofs_north, ofs_east);
}

/*
 *  extrapolate latitude/longitude given distances north and east
 *  This function costs about 80 usec on an AVR2560
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (ofs_north != 0 || ofs_east != 0) {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}


// wrap an angle in degrees to -180..180
int wrap_180(int angle)
{
	if (angle > 3600 || angle < -3600) {
		// for very large numbers use modulus
		angle = angle % 360;
	}
	while (angle > 180) { angle -= 360; }
	while (angle < -180) { angle += 360; }
	return angle;
}

/*
wrap an angle in degrees to 0..359
*/
float wrap_360(float angle)
{
	if (angle > 3600 || angle < -3600) {
		// for very large numbers use modulus
		angle = fmodf(angle, 360);
	}
	while (angle >= 360) angle -= 360;
	while (angle < 0) angle += 360;
	return angle;
}




/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10*PI || angle_in_radians < -10*PI) {
        // for very large numbers use modulus
        angle_in_radians = fmodf(angle_in_radians, 2*PI);
    }
    while (angle_in_radians > PI) angle_in_radians -= 2*PI;
    while (angle_in_radians < -PI) angle_in_radians += 2*PI;
    return angle_in_radians;
}


bool reached_waypoint_point(const struct Location &location,
	const struct Location &point1,
	const struct Location &point2,
	int WP_Boundary)
{
	// function to return a boolean value indicating if we've successfully reached a Waypoint
	// defined by a location and a boundary.
	// V1.0 21/10/2018 

	bool passed_WP = location_passed_point(location, point1, point2);
	bool within_Range = ( (int)get_distance(location, point2) <= WP_Boundary );

	return (passed_WP && within_Range);
};



