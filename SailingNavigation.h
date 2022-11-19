// SailingNavigation.h
// This is where the sailing course is decided.
// The course to be steered is established here, including the decsions to tack.
// The actual course to be steered is maintained in the Course To Steer (CTS).

#ifndef _SAILINGNAVIGATION_h
#define _SAILINGNAVIGATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Navigation.h"
#include "TelemetryLogging.h"

void SailingNavigation_Init(void);
void UpdateCourseToSteer(void);

void UpdateTargetHeading(void);

int CalculateSailingCTS(void);

SteeringCourseType GetFavouredTack(NavigationDataType NavData);

int SetTack(SteeringCourseType Tack, DecisionEventReasonType Reason);
int LimitToSailingCourse(int Course);
int SteerCloseHauled(SteeringCourseType Tack);
int SteerDeepRunning(SteeringCourseType Tack);

bool IsBTWSailable(NavigationDataType NavData);

PointOfSailType GetPointOfSail(int WindAngle);

#endif

