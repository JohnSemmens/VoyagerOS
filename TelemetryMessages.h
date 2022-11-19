// TelemetryMessages.h

#ifndef _TELEMETRYMESSAGES_h
#define _TELEMETRYMESSAGES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum TelMessageType {Dummy_0, PRL, MCP, LNA, LAT, LPO, LWP, LMI, LWI, LSV, LVS, LPF
					, MCC, MIG, MIS, HLG, HLS, VER, TMG, LOG, LCD
					, EQG, CCS, CCG, WC1, WC0, SCS, SCG, DBG
,EndMarker};


void SendMessage(int SerialPortNumber, TelMessageType msg);
void ProcessQueue(int SerialPortNumber);

void QueueMessage(TelMessageType msg);
void SendMissionStep(int CommandPort, int index);

#endif

