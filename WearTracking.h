// WearTracking.h

#ifndef _WEARTRACKING_h
#define _WEARTRACKING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Filters.h"

enum ServoPositionType {
	NotDefined,
	Above,
	Below
};

struct VesselUsageCountersStruct {
	byte init_flag;
	unsigned long intervalCounter;
	unsigned long PortRudderCounter;
	unsigned long StarboardRudderCounter;
	unsigned long TrimTabCounter;
	unsigned long SpareCounter1; // these are here to reserve eeprom space for future counters.
	unsigned long SpareCounter2; // we don't want the addition of any future usage counters
	unsigned long SpareCounter3; // to interfere with existing lifetime usage data.
};


class WearCounter
{
protected:
	int AvgerageServo_us;
	int PreviousServo_us;
	LowPassFilter ServoFilter;
	ServoPositionType PreviousPosition;
	ServoPositionType Position;
public:
	WearCounter()
	{
	};

	float FilterConstant = 0.0001; // default value only
	void TrackServoUsage(int ServoPulse_us);
	unsigned long Counter;
};  

void updateUsageTrackingStats(void);

void init_wearTracking(void);

#endif

