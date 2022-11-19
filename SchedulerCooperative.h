// SchedulerCooperative.h

// This scheduler allows for tasks to be called at regular intervals.
// It does not use interupts, in order to maintain compatibility with libraries that do, such as the wire library.
// V1.0 31/7/2016 John Semmens

#ifndef _SCHEDULERCOOPERATIVE_h
#define _SCHEDULERCOOPERATIVE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void SchedulerTick(int TaskIndex, void(*action)(void*), unsigned long delayMs);

void SchedulerInit(void);

#endif

