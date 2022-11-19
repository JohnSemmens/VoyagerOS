// 
// 
// 

#include "Filters.h"

float LowPassFilter::Filter(float Input)
{
	float filtered_value;
	filtered_value = LowPassFilter::last_filtered_value + LowPassFilter::FilterConstant * (Input - LowPassFilter::last_filtered_value);
	LowPassFilter::last_filtered_value = filtered_value;

	return filtered_value;
};

float HighPassFilter::Filter(float Input)
{
	float filtered_value;
	filtered_value = HighPassFilter::FilterConstant * (Input + HighPassFilter::last_filtered_value - HighPassFilter::last_input);

	HighPassFilter::last_input = Input;
	HighPassFilter::last_filtered_value = filtered_value;

	return filtered_value;
};

void  HighPassFilter::Init(float InitValue)
{
	 last_input = InitValue;
	 last_filtered_value = 0;
};

float LowPassAngleFilter::Filter(float InputAngle)
{
	float filtered_value_X;
	float filtered_value_Y;

	// ensure angle is confined to 0 to  <360
	if (InputAngle >= 360) { InputAngle -= 360; }
	if (InputAngle < 0) { InputAngle += 360; }

	// convert angle to cartesian coordinates
	float Input_X = sin(radians(InputAngle));
	float Input_Y = cos(radians(InputAngle));

	// apply low pass filter to X
	filtered_value_X = LowPassAngleFilter::last_filtered_value_X + LowPassAngleFilter::FilterConstant * (Input_X - LowPassAngleFilter::last_filtered_value_X);
	LowPassAngleFilter::last_filtered_value_X = filtered_value_X;

	// apply low pass filter to Y
	filtered_value_Y = LowPassAngleFilter::last_filtered_value_Y + LowPassAngleFilter::FilterConstant * (Input_Y - LowPassAngleFilter::last_filtered_value_Y);
	LowPassAngleFilter::last_filtered_value_Y = filtered_value_Y;

	// convert filtered angle components back to an angle between 0 and <360 degrees.
	float filtered_angle_value = degrees(atan2(filtered_value_X, filtered_value_Y));
	if (filtered_angle_value < 0) { filtered_angle_value += 360; }

	return filtered_angle_value;
};





