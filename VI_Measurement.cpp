// 
// 
// 

#include "VI_Measurement.h"
#include "Adafruit_INA219.h"

extern Adafruit_INA219 ina219;
extern double PowerSensorV, PowerSensorI ; 

void init_VI_Measurement(void)
{
	ina219.begin(); // I2C Address 0x40
	ina219.setCalibration_16V_400mA();
}

void read_VI_Measurement(void)
{
	// Read the INA219 I2C Voltage/Current Sensor
	PowerSensorV = ina219.getBusVoltage_V();
	PowerSensorI = ina219.getCurrent_mA();
}