// Loiter.h

#ifndef _LOITER_h
#define _LOITER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Waypoints.h"

enum LoiterStateType {
	lsNotLoitering,			// Not Loitering
	lsApproach,				// Heading to Loiter Waypoint prior to commencing loitering
	//lsOutbound,				// Loitering - moving away from loiter point
	//lsInbound,				// Loitering - moving toward loiter point
	lsPortSide,				// Loitering - reaching toward port side
	lsStarboardSide,		// Loitering - reaching toward starboard side
};

struct LoiterStruct {
	LoiterStateType LoiterState; // This describes the current loiter state, approach, outbound inbound
	Location loiterCentreLocation;		// The Loiter Location when the Vessel Command "loiter Here" is applied.
//	Location loiterOuterLocation;	// the outer boundary location when transitioning from outbound to inbound while loitering
	Location loiterLocationPrevious; // Previous Loiter Location
	Location LoiterLocationNext;
	Location PortSideLocation;		// Port-Side loiter Location
	Location StarboardSideLocation;		// Starboard-Side loiter Location
	long DTW;			 // Distance to Loiter Waypoint - metres
	int BTW;			 // Bearing to Loiter Waypoint - Degrees from true North
	bool PastWP;		 // past the Loiter Waypoint -  True/False
	long LoiterRadius;	
};

int LoiterCalcCTS(void);


#endif

