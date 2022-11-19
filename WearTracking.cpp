// 
// 
// 

#include "WearTracking.h"
#include "Filters.h"
#include "configValues.h"

extern WearCounter PortRudderUsage;
extern WearCounter StarboardRudderUsage;
extern WearCounter TrimTabUsage;
extern VesselUsageCountersStruct VesselUsageCounters;

// Note: it would be possible to

void init_wearTracking(void)
{
	// init the counters from stored values in the EEPROM to keep more of a life time value rather mission time.
	 Load_EEPROM_VesselUsage();
};

void updateUsageTrackingStats(void)
{
	// This is called at about one second intervals to update the main usage tracking object with the individual
	// usage tracking counters.
	VesselUsageCounters.PortRudderCounter = PortRudderUsage.Counter;
	VesselUsageCounters.StarboardRudderCounter = StarboardRudderUsage.Counter;
	VesselUsageCounters.TrimTabCounter = TrimTabUsage.Counter;
};


void WearCounter::TrackServoUsage(int ServoPulse_us)
{
	// pass servo pulse width through a low pass filter 
	AvgerageServo_us = int(ServoFilter.Filter(float(ServoPulse_us)));

	const int Deadband = 10;

		if (ServoPulse_us > (AvgerageServo_us + Deadband))
		{
			Position = ServoPositionType::Above;
		};

		if (ServoPulse_us < (AvgerageServo_us - Deadband))
		{
			Position = ServoPositionType::Below;
		};

		if (PreviousPosition != Position)
		{
			Counter++;
		}

	PreviousPosition = Position;
}