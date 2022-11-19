/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "USFS_IMU.h"
#include "location.h"

extern float qt[2][4];
extern int16_t  accADC[2][3];
extern uint8_t eventStatus[2];
extern uint8_t algostatus[2];
extern uint8_t Quat_flag[2], Gyro_flag[2], Acc_flag[2], Mag_flag[2], Baro_flag[2];
extern int16_t gyroADC[2][3], acc[2][3], accADC[2][3], acc_calADC[2][3], magADC[2][3];
extern int16_t rawPressure[2], rawTemperature[2];
extern float gyroData[2][3];
extern float LINaccData[2][3];
extern float magData[2][3];
extern float accData[2][3];
extern float pressure[2];
extern float temperature[2];

extern IMUStruct	myIMU;
extern I2Cdev      *i2c_0;
extern EM7180      *Sentral_0;
extern IMU         *imu_0;
extern HardwareSerial *Serials[];


// Declare main loop helper functions
void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM);

void IMU_Init(int CommandPort)
{
	// imu wrapper for initialising the USFS IMU.
	// V1.0 25/4/2019  Created by John Semmens

	//(*Serials[CommandPort]).println(F("MSG,IMU BNO080 detected at 0x4B"));

	// Instantiate Sentral_0 classes and create function pointers
	i2c_0 = new I2Cdev(&Wire);   // Use class instance/function pointer to specify I2C bus (may be more than one)
	Sentral_0 = new EM7180(i2c_0, 0);                                                                                  // Use class instance/function pointer to specify Sentral board (may be more than one)
	imu_0 = new IMU(Sentral_0, 0);                                                                                 // Use class instance/function pointer to specify the Sentral and I2C instances for the IMU calcs	
	(*Serials[CommandPort]).println(F("MSG,Initializing Sentral_0"));
	Sentral_0->initSensors();
};

void IMU_Read(void)
{
	// IMU wrapper function to call the specific IMU attitude information for the USFS IMU.
	// V1.0 25/4/2019  Created by John Semmens

	// Acquire data the Sentral
	FetchEventStatus(i2c_0, 0);
	FetchSentralData(Sentral_0, imu_0, 0);

	myIMU.heading = wrap_360(heading[0]);
	myIMU.pitch = angle[0][1];
	myIMU.roll = angle[0][0];

	myIMU.baro_pressure = pressure[0];
	myIMU.baro_temperature = temperature[0];
	myIMU.Algorithm_Status = algostatus[0];
};

void IMU_Save_Cal(void)
{
	// save the current compass cal factors to EEPROM in the USFS IMU.
	// These are for later recall with warm start
	Sentral_0->Save_Sentral_WS_params();
}

void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
	eventStatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_EventStatus);
	if (eventStatus[sensorNUM] & 0x04) Quat_flag[sensorNUM] = 1;
	if (eventStatus[sensorNUM] & 0x20) Gyro_flag[sensorNUM] = 1;
	if (eventStatus[sensorNUM] & 0x10) Acc_flag[sensorNUM] = 1;
	if (eventStatus[sensorNUM] & 0x08) Mag_flag[sensorNUM] = 1;
	if (eventStatus[sensorNUM] & 0x40) Baro_flag[sensorNUM] = 1;
	algostatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
}

void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM)
{
	if (Gyro_flag[sensorNUM] == 1)
	{
		em7180->Gyro_getADC();
		for (uint8_t i = 0; i<3; i++)
		{
			gyroData[sensorNUM][i] = (float)gyroADC[sensorNUM][i] * DPS_PER_COUNT;
		}
		Gyro_flag[sensorNUM] = 0;
	}
	if (Quat_flag[sensorNUM] == 1)
	{
		IMu->computeIMU();
		Quat_flag[sensorNUM] = 0;
	}
	if (Acc_flag[sensorNUM])
	{
		em7180->ACC_getADC();
		em7180->ACC_Common();
		for (uint8_t i = 0; i<3; i++)
		{
			accData[sensorNUM][i] = (float)accADC[sensorNUM][i] * G_PER_COUNT;
			LINaccData[sensorNUM][i] = (float)accLIN[sensorNUM][i] * G_PER_COUNT;
		}
		Acc_flag[sensorNUM] = 0;
	}
	if (Mag_flag[sensorNUM])
	{
		em7180->Mag_getADC();
		for (uint8_t i = 0; i<3; i++)
		{
			magData[sensorNUM][i] = (float)magADC[sensorNUM][i] * SENTRAL_UT_PER_COUNT;
		}
		Mag_flag[sensorNUM] = 0;
	}
	if (Baro_flag[sensorNUM])
	{
		rawPressure[sensorNUM] = em7180->Baro_getPress();
		pressure[sensorNUM] = (float)rawPressure[sensorNUM] * 0.01f + 1013.25f;                                       // Pressure in mBar
		rawTemperature[sensorNUM] = em7180->Baro_getTemp();
		temperature[sensorNUM] = (float)rawTemperature[sensorNUM] * 0.01;                                               // Temperature in degrees C
		Baro_flag[sensorNUM] = 0;
	}
}



IMU::IMU(EM7180* sentral, uint8_t sensornum)
{
  Sentral = sentral;
  SensorNum = sensornum;
}

/**
* @fn: computeIMU ();
*
* @brief: Calculates state estimate
* 
* @params: 
* @returns: 
*/
void IMU::computeIMU ()
{
  float a11[2], a21[2], a31[2], a32[2], a33[2];
  float yaw[2];
  static float buff_roll[2] = {0.0f, 0.0f}, buff_pitch[2] = {0.0f, 0.0f}, buff_heading[2] = {0.0f, 0.0f};

  // Pass-thru for future filter experimentation
  accSmooth[SensorNum][0] = accADC[SensorNum][0];
  accSmooth[SensorNum][1] = accADC[SensorNum][1];
  accSmooth[SensorNum][2] = accADC[SensorNum][2];

  Sentral->getQUAT();
 
  // Only five elements of the rotation matrix are necessary to calculate the three Euler angles
  a11[SensorNum] = qt[SensorNum][0]*qt[SensorNum][0]+qt[SensorNum][1]*qt[SensorNum][1]
                   -qt[SensorNum][2]*qt[SensorNum][2]-qt[SensorNum][3]*qt[SensorNum][3];
  a21[SensorNum] = 2.0f*(qt[SensorNum][0]*qt[SensorNum][3]+qt[SensorNum][1]*qt[SensorNum][2]);
  a31[SensorNum] = 2.0f*(qt[SensorNum][1]*qt[SensorNum][3]-qt[SensorNum][0]*qt[SensorNum][2]);
  a32[SensorNum] = 2.0f*(qt[SensorNum][0]*qt[SensorNum][1]+qt[SensorNum][2]*qt[SensorNum][3]);
  a33[SensorNum] = qt[SensorNum][0]*qt[SensorNum][0]-qt[SensorNum][1]*qt[SensorNum][1]
                   -qt[SensorNum][2]*qt[SensorNum][2]+qt[SensorNum][3]*qt[SensorNum][3];

  // Pass-thru for future filter experimentation
  buff_roll[SensorNum]    = (atan2(a32[SensorNum], a33[SensorNum]))*(57.2957795f);                                                          // Roll Right +ve
  buff_pitch[SensorNum]   = -(asin(a31[SensorNum]))*(57.2957795f);                                                                          // Pitch Up +ve
  buff_heading[SensorNum] = (atan2(a21[SensorNum], a11[SensorNum]))*(57.2957795f);                                                          // Yaw CW +ve
  
  angle[SensorNum][0] = buff_roll[SensorNum];
  angle[SensorNum][1] = buff_pitch[SensorNum];
  yaw[SensorNum]      = buff_heading[SensorNum];
  heading[SensorNum]  = yaw[SensorNum] + MAG_DECLINIATION;
  if(heading[SensorNum] < 0.0f) 
	  heading[SensorNum] += 360.0f;                                                                               // Convert heading to 0 - 360deg range

}
