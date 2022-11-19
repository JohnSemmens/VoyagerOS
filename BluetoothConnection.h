// BluetoothConnection.h

#ifndef _BLUETOOTHCONNECTION_h
#define _BLUETOOTHCONNECTION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

enum BTStateType { Idle, Initialising, Initialised, Querying, Connecting, Connected };

void Bluetooth_Init(int BluetoothPort);

void BluetoothInitialiseConnection(int BluetoothPort);

void BluetoothManageConnection(int BluetoothPort);

char * GetBTStatus(BTStateType BTState);

void BT_CLI_Process_Message(int BluetoothPort);
void BT_CLI_Processor(int BluetoothPort);

#endif

