// OLED_Logging.h

#ifndef _OLED_LOGGING_h
#define _OLED_LOGGING_h

#include "Mission.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

void OLED_Init(int SerialPortNumber);
void OLED_Logging(char LoggingLevel);
String GetMissionCommandString(MissionCommandType cmd);

#endif

