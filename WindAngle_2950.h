// WindAngle_2950.h

#ifndef _WindAngle_2950_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void WingSailAngleSensor_Init(int CommandPort);
void WingSailAngleSensor_Read(void);

void IMU_Mag_Init(void);
void IMU_Mag_Calibrate(void);

// load the config values relating to the IMU, into the IMU.
void Load_IMU_Calibation_Values(void);

#endif

