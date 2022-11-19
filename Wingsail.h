// Wingsail.h

#ifndef _WINGSAIL_h
#define _WINGSAIL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum WingSailTackType {
	wsHeadToWind,
	wsPortTack,
	wsStarboardTack
};

enum WingSailStateType {
	wsIdle,
	wsForward,
	wsReverse
};

struct WingSailType {
	int Angle;
	int TrimTabAngle;
	WingSailStateType State;
	int Servo_microseconds;
	WingSailTackType Tack;
};

void wingsail_init(void);

void wingsail_update(void);
void AutoSetWingSail(WingSailStateType WingSailState);
void SetTrimTabAngle(int TrimTabAngle);

int CalcTrimTabAngle(int WingSailAngle, WingSailStateType WingSailState);

int TrimTabAngle_to_us(int TrimTabAngle);

void WingSailServo(int SailIn_us);

void IdentifyCurrentTack(int WingSailAngle);

void Wingsail_TrackTackChange(void);

#endif

