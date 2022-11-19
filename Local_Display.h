// Local_Display.h

#ifndef _LOCAL_DISPLAY_h
#define _LOCAL_DISPLAY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

void Local_Display_Logging(char LoggingLevel);

#endif

