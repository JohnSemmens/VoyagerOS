// Filters.h

#ifndef _FILTERS_h
#define _FILTERS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class LowPassFilter
{
	protected:
		float last_filtered_value;
		
	public:
		float FilterConstant = 0.05; // default value only
		float Filter(float Input);

};  // class LowPassFilter

class HighPassFilter
{
protected:
	float last_input;
	float last_filtered_value;

public:
	float FilterConstant = 0.95; // default value only
	float Filter(float Input);
	void Init(float InitValue);

};  // class HighPassFilter


class LowPassAngleFilter
{
protected:
	float last_filtered_value_X;
	float last_filtered_value_Y;

public:
	float FilterConstant = 0.05; // default value only
	float Filter(float Input);

};  // class LowPassAngleFilter



#endif

