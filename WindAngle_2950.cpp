// 
// 
// 
#include "WindAngle_2950.h"

/* MPU9250 Basic Example Code
by: Kris Winer
date: April 1, 2014
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
Modified by Brent Wilkins July 19, 2016

Demonstrate basic MPU-9250 functionality including parameterizing the register
addresses, initializing the sensor, getting properly scaled accelerometer,
gyroscope, and magnetometer data out. Added display functions to allow display
to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
and the Teensy 3.1.

SDA and SCL should have external pull-up resistors (to 3.3V).
10k resistors are on the EMSENSR-9250 breakout board.

Hardware setup:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
VDDI --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND
*/

#include <Wire.h>
#include "WindAngle_2950.h"
#include "MPU9250.h"
#include "location.h"

#include "configValues.h"

const byte MPU9250_ID = 0x71;
const byte MPU9255_ID = 0x73;

// Pin definitions
//int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

extern MPU9250 WingSailAngleSensor;
extern configValuesType Configuration;
extern HardwareSerial *Serials[];

void WingSailAngleSensor_Init(int CommandPort)
{
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 22/11/2016 updated to improve handling where there's no IMU found.
	// V1.3 17/3/2019 updatd to support both MPU 2950 and 2955. I think they'll operate the same but different IDs.

	// TWBR = 12;  // 400 kbit/sec I2C speed
	Wire.begin();
	// Set up the interrupt pin, its set as active high, push-pull
	//pinMode(intPin, INPUT);
	//digitalWrite(intPin, LOW);

	// ensure calibration mode is off
	WingSailAngleSensor.MagneticAngleCalibrationMode = false;

	// Read the WHO_AM_I register, this is a good test of communication
	byte c = WingSailAngleSensor.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	if (c == MPU9250_ID || c == MPU9255_ID) // WHO_AM_I should always be 0x68
	{
		if (c == MPU9250_ID)
		{
			(*Serials[CommandPort]).print(F("MSG,MPU9250 found at 0x"));
		} 
		else
		{
			(*Serials[CommandPort]).print(F("MSG,MPU9255 found at 0x"));

		}
		(*Serials[CommandPort]).println(MPU9250_ADDRESS, HEX);
		(*Serials[CommandPort]).print(F("MSG,MPU ID "));
		(*Serials[CommandPort]).println(c, HEX);

		//(*Serials[CommandPort]).println(F("MSG,MPU9250 Debug 1"));
		// Start by performing self test and reporting values
		WingSailAngleSensor.MPU9250SelfTest(WingSailAngleSensor.SelfTest);
	//	(*Serials[CommandPort]).println(F("MSG,MPU9250 Debug 2"));

		//(*Serials[CommandPort]).print("x-axis self test: acceleration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[0], 1); (*Serials[CommandPort]).println("% of factory value");
		//(*Serials[CommandPort]).print("y-axis self test: acceleration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[1], 1); (*Serials[CommandPort]).println("% of factory value");
		//(*Serials[CommandPort]).print("z-axis self test: acceleration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[2], 1); (*Serials[CommandPort]).println("% of factory value");
		//(*Serials[CommandPort]).print("x-axis self test: gyration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[3], 1); (*Serials[CommandPort]).println("% of factory value");
		//(*Serials[CommandPort]).print("y-axis self test: gyration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[4], 1); (*Serials[CommandPort]).println("% of factory value");
		//(*Serials[CommandPort]).print("z-axis self test: gyration trim within : ");
		//(*Serials[CommandPort]).print(WingSailAngleSensor.SelfTest[5], 1); (*Serials[CommandPort]).println("% of factory value");


		// Calibrate gyro and accelerometers, load biases in bias registers
			WingSailAngleSensor.calibrateMPU9250(WingSailAngleSensor.gyroBias, WingSailAngleSensor.accelBias);

		//	(*Serials[CommandPort]).println(F("MSG,MPU9250 Debug 3"));

			WingSailAngleSensor.initMPU9250();
		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature

		// Read the WHO_AM_I register of the magnetometer, this is a good test of
		// communication
	//	byte d = WingSailAngleSensor.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
		//Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
		//Serial.print(" I should be "); Serial.println(0x48, HEX);

	//	(*Serials[CommandPort]).println(F("MSG,MPU9250 Debug 4"));

		// Get magnetometer calibration from AK8963 ROM
		WingSailAngleSensor.initAK8963(WingSailAngleSensor.magCalibration);

	//	(*Serials[CommandPort]).println(F("MSG,MPU9250 Debug 5"));

		// Initialize device for active mode read of magnetometer
		(*Serials[CommandPort]).println(F("MSG,AK8963 initialized for active data mode...."));
		(*Serials[CommandPort]).println(F("MSG,MPU9250 initialized for active data mode...."));

		//	//  Serial.println("Calibration values: ");
		//(*Serials[CommandPort]).print("X-Axis sensitivity adjustment value ");
		//(*Serials[CommandPort]).println(WingSailAngleSensor.magCalibration[0], 2);
		//(*Serials[CommandPort]).print("Y-Axis sensitivity adjustment value ");
		//(*Serials[CommandPort]).println(WingSailAngleSensor.magCalibration[1], 2);
		//(*Serials[CommandPort]).print("Z-Axis sensitivity adjustment value ");
		//(*Serials[CommandPort]).println(WingSailAngleSensor.magCalibration[2], 2);

		//IMU_Mag_Init();					// either init the calibration values from a reading; or
		Load_IMU_Calibation_Values();	// load the calibration values from config

		WingSailAngleSensor.Initialsed = true;
	} // if (c == 0x71 or 0x73)
	else
	{
		(*Serials[CommandPort]).print(F("MSG,No IMU MPU9250 found: 0x"));
		(*Serials[CommandPort]).println(MPU9250_ADDRESS, HEX);

		WingSailAngleSensor.Initialsed = false;
	}
}

void IMU_Mag_Init(void)
{
	// This is used as a prelude to calibration.
	// It is intended to be used once to initiate the compass calibration limits to safe values prior to a calibration run.
	// V1.0 15/8/2016 John Semmens

	WingSailAngleSensor_Read(); // ensure that the IMU Mag values are reasonable before clearing and re-initiating the recorded limits

	WingSailAngleSensor.mx_min = WingSailAngleSensor.mx;
	WingSailAngleSensor.mx_max = WingSailAngleSensor.mx;
	WingSailAngleSensor.my_min = WingSailAngleSensor.my;
	WingSailAngleSensor.my_max = WingSailAngleSensor.my;
	WingSailAngleSensor.mz_min = WingSailAngleSensor.mz;
	WingSailAngleSensor.mz_max = WingSailAngleSensor.mz;
}

void IMU_Mag_Calibrate(void)
{
	// This is called after each compass read during a calibration run.
	// The IMU mx, my & mz values follow an approximate sine wave (with an amplitude an offset to be calibated out).
	// This function will adjust the upper and lower limits of the sine curve of each axis.
	// it is intended that that vessel is placed manually into compass calibration mode, and then about two full turns are performed.
	// This should set resonable upper and lower limits for each axis.
	// These should then be recorded and set in the config stucture EEPROM.
	// V1.0 15/8/2016 John Semmens

	// extend calibration limits
	if (WingSailAngleSensor.mx_min > WingSailAngleSensor.mx) WingSailAngleSensor.mx_min = WingSailAngleSensor.mx;
	if (WingSailAngleSensor.mx_max < WingSailAngleSensor.mx) WingSailAngleSensor.mx_max = WingSailAngleSensor.mx;

	if (WingSailAngleSensor.my_min > WingSailAngleSensor.my) WingSailAngleSensor.my_min = WingSailAngleSensor.my;
	if (WingSailAngleSensor.my_max < WingSailAngleSensor.my) WingSailAngleSensor.my_max = WingSailAngleSensor.my;

	if (WingSailAngleSensor.mz_min > WingSailAngleSensor.mz) WingSailAngleSensor.mz_min = WingSailAngleSensor.mz;
	if (WingSailAngleSensor.mz_max < WingSailAngleSensor.mz) WingSailAngleSensor.mz_max = WingSailAngleSensor.mz;

	//save to configuration object
	Configuration.mx_min = WingSailAngleSensor.mx_min;
	Configuration.mx_max = WingSailAngleSensor.mx_max;
	Configuration.my_min = WingSailAngleSensor.my_min;
	Configuration.my_max = WingSailAngleSensor.my_max;
	Configuration.mz_min = WingSailAngleSensor.mz_min;
	Configuration.mz_max = WingSailAngleSensor.mz_max;
}

void WingSailAngleSensor_Read(void)
{
	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	if (WingSailAngleSensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		WingSailAngleSensor.readAccelData(WingSailAngleSensor.accelCount);  // Read the x/y/z adc values
		WingSailAngleSensor.getAres();

		// Now we'll calculate the accleration value into actual g's
		// This depends on scale being set
		WingSailAngleSensor.ax = (float)WingSailAngleSensor.accelCount[0] * WingSailAngleSensor.aRes; // - accelBias[0];
		WingSailAngleSensor.ay = (float)WingSailAngleSensor.accelCount[1] * WingSailAngleSensor.aRes; // - accelBias[1];
		WingSailAngleSensor.az = (float)WingSailAngleSensor.accelCount[2] * WingSailAngleSensor.aRes; // - accelBias[2];

		WingSailAngleSensor.readGyroData(WingSailAngleSensor.gyroCount);  // Read the x/y/z adc values
		WingSailAngleSensor.getGres();

		// Calculate the gyro value into actual degrees per second
		// This depends on scale being set
		WingSailAngleSensor.gx = (float)WingSailAngleSensor.gyroCount[0] * WingSailAngleSensor.gRes;
		WingSailAngleSensor.gy = (float)WingSailAngleSensor.gyroCount[1] * WingSailAngleSensor.gRes;
		WingSailAngleSensor.gz = (float)WingSailAngleSensor.gyroCount[2] * WingSailAngleSensor.gRes;

		WingSailAngleSensor.readMagData(WingSailAngleSensor.magCount);  // Read the x/y/z adc values
		WingSailAngleSensor.getMres();
		// User environmental x-axis correction in milliGauss, should be
		// automatically calculated
		//myIMU.magbias[0] = (mx_max - mx_min) / 2; // +470.;
		//// User environmental x-axis correction in milliGauss TODO axis??
		//myIMU.magbias[1] = (my_max - my_min) / 2; // +120.;
		//// User environmental x-axis correction in milliGauss
		//myIMU.magbias[2] = (mz_max - mz_min) / 2; // +125.;

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental
		// corrections
		// Get actual magnetometer value, this depends on scale being set
		WingSailAngleSensor.mx = (float)WingSailAngleSensor.magCount[0] * WingSailAngleSensor.mRes*WingSailAngleSensor.magCalibration[0] - WingSailAngleSensor.magbias[0];
		WingSailAngleSensor.my = (float)WingSailAngleSensor.magCount[1] * WingSailAngleSensor.mRes*WingSailAngleSensor.magCalibration[1] - WingSailAngleSensor.magbias[1];
		WingSailAngleSensor.mz = (float)WingSailAngleSensor.magCount[2] * WingSailAngleSensor.mRes*WingSailAngleSensor.magCalibration[2] - WingSailAngleSensor.magbias[2];


		float mx_cal = (WingSailAngleSensor.mx - (WingSailAngleSensor.mx_max + WingSailAngleSensor.mx_min) / 2) / (WingSailAngleSensor.mx_max - WingSailAngleSensor.mx_min);
		float my_cal = (WingSailAngleSensor.my - (WingSailAngleSensor.my_max + WingSailAngleSensor.my_min) / 2) / (WingSailAngleSensor.my_max - WingSailAngleSensor.my_min);
		float mz_cal = (WingSailAngleSensor.mx - (WingSailAngleSensor.mz_max + WingSailAngleSensor.mz_min) / 2) / (WingSailAngleSensor.mz_max - WingSailAngleSensor.mz_min);
		WingSailAngleSensor.MagneticAngle = atan2(my_cal, mx_cal) * RAD_TO_DEG;

		// simple Pitch and Roll calculation. No Yaw.
		WingSailAngleSensor.roll = atan2(WingSailAngleSensor.ax, WingSailAngleSensor.az)  * RAD_TO_DEG;
		WingSailAngleSensor.pitch = atan2(WingSailAngleSensor.ay, WingSailAngleSensor.az) * RAD_TO_DEG;

	} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

	  // Must be called before updating quaternions!
	WingSailAngleSensor.updateTime();

	WingSailAngleSensor.delt_t = millis() - WingSailAngleSensor.count;

	WingSailAngleSensor.tempCount = WingSailAngleSensor.readTempData();  // Read the adc values
												// Temperature in degrees Centigrade
	WingSailAngleSensor.temperature = ((float)WingSailAngleSensor.tempCount) / 333.87 + 21.0;

	WingSailAngleSensor.count = millis();
	WingSailAngleSensor.sumCount = 0;
	WingSailAngleSensor.sum = 0;

	if (WingSailAngleSensor.MagneticAngleCalibrationMode == true)
			IMU_Mag_Calibrate();
}



void Load_IMU_Calibation_Values(void)
{
	// load the config values relating to the IMU, into the IMU.
	// V1.0 27/8/2016 John Semmens

	WingSailAngleSensor.mx_min = Configuration.mx_min;
	WingSailAngleSensor.mx_max = Configuration.mx_max;
	WingSailAngleSensor.my_min = Configuration.my_min;
	WingSailAngleSensor.my_max = Configuration.my_max;
	WingSailAngleSensor.mz_min = Configuration.mz_min;
	WingSailAngleSensor.mz_max = Configuration.mz_max;
}