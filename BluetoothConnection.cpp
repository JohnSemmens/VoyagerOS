// 
// 
// 

#include "BluetoothConnection.h"
#include "configValues.h"

extern byte BluetoothStatePin;
extern BTStateType BTState;
extern HardwareSerial *Serials[];
extern configValuesType Configuration;
extern int CommandPort;


void Bluetooth_Init(int BluetoothPort)
{
	// called from setup()

	(*Serials[BluetoothPort]).begin(Configuration.BTPortBaudRate);
	pinMode(BluetoothStatePin, INPUT_PULLUP);
}

void BluetoothManageConnection(int BluetoothPort)
{
	// call this connection management procedure at around 5 second intervals

	static BTStateType BTStatePrev;

	if (digitalRead(BluetoothStatePin) == LOW) // LOW is not connected
	{
		// if not connected then proceed with connection process
		switch (BTState)
		{
		case Idle:
		case Initialising:
			BluetoothInitialiseConnection(BluetoothPort);
			break;
		case Initialised:
			(*Serials[BluetoothPort]).println(F("AT+INQ"));
			BTState = Querying;
			break;
		case Querying:
			(*Serials[BluetoothPort]).println(F("AT+CONN1"));
			BTState = Connecting;
			break;
		case Connecting:
			BTState = Connected;
			break;
		case Connected:
			// if the current state is "Connected", but now the Connected Status line has dropped
			// then it indicates that we had a connection, but its dropped.
			// hence move the state back to "initialised" to force a reconnection.
			if (digitalRead(BluetoothStatePin) == LOW)
			{
				BTState = Initialised;
			}
		default:;
		};
	}
	else
	{
		// BluetoothStatePin is high
		BTState = Connected;
	}

	// look for a change in Bluetooth State and then log a message
	if (BTStatePrev != BTState)
	{
	//	(*Serials[CommandPort]).print(F("MSG,BT "));
	//	(*Serials[CommandPort]).println(GetBTStatus(BTState));

		if (BTState == BTStateType::Connected)
		{
			(*Serials[Configuration.BluetoothPort]).println(); // send a line return to clear the buffer
			(*Serials[Configuration.BluetoothPort]).println(F("ver"));
			(*Serials[Configuration.BluetoothPort]).println(F("gvi"));
		}
	}
	BTStatePrev = BTState;
};

void BluetoothInitialiseConnection(int BluetoothPort)
{
	// Initialise the Bluetooth module
	// Call this procedure repeatedly with a minimum of 100ms period between calls

	static int BTInitStep;

	switch (BTInitStep)
	{
	case 0:
		(*Serials[BluetoothPort]).println("AT");
		BTInitStep = 1;
		BTState = Initialising;
		break;
	case 1:
		(*Serials[BluetoothPort]).println(F("AT+RENEW"));
		BTInitStep = 2;
		break;
	case 2:
		(*Serials[BluetoothPort]).println(F("AT+RESET"));
		BTInitStep = 3;
		break;
	case 3:
		(*Serials[BluetoothPort]).println(F("AT+NAMEVOYMASTER"));
		BTInitStep = 4;
		break;
	case 4:
		(*Serials[BluetoothPort]).println(F("AT+ROLE1"));
		BTState = Initialised;
		break;
	default:
		BTInitStep = 0;
	}
}

char * GetBTStatus(BTStateType BTState)
{
	// return a string representation of the Bluetooth state

	switch ( BTState)
	{
	case Idle:			return "Idle";
	case Initialising:	return "Initialising";
	case Initialised:	return "Initialised";
	case Querying:		return "Querying";
	case Connecting:	return "Connecting";
	case Connected:		return "Connected";
	default:			return "unknown";
	};
}



// Command Line Interpreter - Global Variables
char BT_CLI_Msg[50];
int BT_CLI_i = 0;

// Collect the characters into a command string until end end of line,
// and then process it.
// V1.0 22/12/2015
void BT_CLI_Process_Message(int BluetoothPort)
{
	// Accumulate characters in a command string up to a CR or LF or buffer fills. 
	while ((*Serials[BluetoothPort]).available())
	{
		//
		char received = (*Serials[BluetoothPort]).read();
		BT_CLI_Msg[BT_CLI_i++] = received;

		// Process message when new line character is received
		if (received == '\n' || received == '\r' || 1 == sizeof(BT_CLI_Msg) - 1)
		{
			BT_CLI_Msg[BT_CLI_i] = '\0';
			BT_CLI_Processor(BluetoothPort);
			BT_CLI_i = 0;
		}
	}
}


// Process the BT Response String.
// Split into command and parameters separated by commas. 
// V1.0 22/12/2015
// V1.1 13/01/2018 added support for a Vessel Command Parameter. i.e. steer a magnetic heading
// V1.2 1/12/2018 changed strcpy to strncpy to guard against corrupting memory with long strings

void BT_CLI_Processor(int BluetoothPort)
{
	char cmd[4] = "";
	char param1[12] = "";
	char param2[12] = "";
	char param3[12] = "";
	char param4[12] = "";
	char param5[12] = "";
	char param6[12] = "";

	strcat(BT_CLI_Msg, ",");

	// Split into command and parameters separated by commas. 
	strncpy(cmd, strtok(BT_CLI_Msg, ","), sizeof(cmd)-1);
	strncpy(param1, strtok(NULL, ","), sizeof(param1)-1);
	strncpy(param2, strtok(NULL, ","), sizeof(param2)-1);
	strncpy(param3, strtok(NULL, ","), sizeof(param3)-1);
	strncpy(param4, strtok(NULL, ","), sizeof(param4)-1);
	strncpy(param5, strtok(NULL, ","), sizeof(param5)-1);
	strncpy(param6, strtok(NULL, ","), sizeof(param6)-1);



	// ===============================================
	// Wingsail Command ech: Echo the command and parameters
	// ===============================================
	if (!strncmp(cmd, "ech", 3))
	{
		(*Serials[CommandPort]).print(cmd);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param1);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param2);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param3);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param4);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param5);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param6);
		(*Serials[CommandPort]).println();
	}


	// ===============================================
	// Wingsail Command wsv, WingSail Voltage.
	// ===============================================
	// Parameter 1: WingSail Battery Voltage
	// Parameter 2: WingSail Battery currrent
	// 
	if (!strncmp(cmd, "gvi", 3))
	{
		(*Serials[CommandPort]).print(F("wsc"));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(F("gvi,"));
		(*Serials[CommandPort]).print(param1);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param2);
		//(*Serials[CommandPort]).println();
	}

	if (!strncmp(cmd, "srv", 3))
	{
		(*Serials[CommandPort]).print(F("wsc"));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(F("srv"));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param1);
	//	(*Serials[CommandPort]).println();
	}

	if (!strncmp(cmd, "ver", 3))
	{
		(*Serials[CommandPort]).print(F("wsc"));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(F("ver"));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param1);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(param2);
	//	(*Serials[CommandPort]).println();
	}

};