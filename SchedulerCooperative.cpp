// This scheduler allows for tasks to be called at regular intervals.
// It does not use interupts, in order to maintain compatibility with libraries that do, such as the wire library.
// These functions rely on the millis() function, which provides the number of ms since the arduino was started.
// This rolls over after about 50 days, but this should have no affect on this scheduler.
// V1.0 31/7/2016 John Semmens

#include "SchedulerCooperative.h"

// nominate a maximum number of tasks to be supported.
static const long MaxNumberOfTasks = 6;


// array holding the times in ms for the next running of each task.
unsigned long TaskList[MaxNumberOfTasks];

// The SchedulerTick function should be called regularly to see if it is time to run each task.
// This should be called in the LOOP for each Task to be run.
void SchedulerTick(int TaskIndex, void(*action)(void*), unsigned long interval_ms) {
	// TaskIndex: index to Task/function being processed.
	// action: task/function name to be run if its time to run.
	// interval_ms: task running interval in milliseconds. e.g 1000 ms is the interval of one second.

	// example: SchedulerTick(0,&loop_1s,1000);

	unsigned long currentTime = millis();

	if (currentTime > TaskList[TaskIndex])
	{
		TaskList[TaskIndex] = currentTime + interval_ms;
		action(NULL);
	}
}


// Initialise the Task time array to zero. 
// This should called once in the SETUP
void SchedulerInit(void) {
	memset(TaskList, 0, sizeof(TaskList));
}