// SDCardLogFile.h

/*
SD card read / write

This example shows how to read and write data to and from an SD card file
The circuit :
*SD card attached to SPI bus as follows :
** MOSI - pin 11
* * MISO - pin 12
* * CLK - pin 13
* * CS - pin 10

*/


#ifndef _SDCARDLOGFILE_h
#define _SDCARDLOGFILE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
#include "TelemetryLogging.h"

void SD_Logging_Init(int CommandPort);
void SD_Logging(word LoggingMask);

void OpenSDLogFile(String LogFileName);
//void WriteSDLogFileLine(String LogFileTextLine);

void Check_LogFileSize(int CommandPort, word LoggingMask);
void SD_Logging_OpenFile(int CommandPort, word LoggingMask);

//void SD_Logging_Event_Mission(word LoggingMask);
void SD_Logging_Event_Decisions(word LoggingMask);
void SD_Logging_Event_Usage(word LoggingMask);
void SD_Logging_Event_ParameterChange(int ParameterIndex, char ParameterValue[12]);
void SD_Logging_Event_Messsage(String message);
void SD_Logging_Event_MissionStep(int mission_index);

void CloseThenOpenLogFile(word LoggingMask);
void Check_GPS_TimeStatus(word LoggingMask);

#endif

