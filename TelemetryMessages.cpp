// 
// 
// 

#include "TelemetryMessages.h"
#include "Navigation.h"
#include "USFS_IMU.h"
#include "TelemetryLogging.h"
#include "Wingsail.h"
#include "Mission.h"
#include "Wingsail.h"
#include "WindAngle_2950.h"
#include "MPU9250.h"
#include "configValues.h"
#include "GPS.h"
#include "CLI.h"

extern HardwareSerial* Serials[];
extern NavigationDataType NavData;
extern IMUStruct myIMU;
extern byte MessageArray[EndMarker];
extern double PowerSensorV, PowerSensorI;
extern StateValuesStruct StateValues;
extern WingSailType WingSail;
extern double SteeringServoOutput;
extern MissionValuesStruct MissionValues;
extern char Version[];
extern char VersionDate[];
extern Time CurrentLocalTime;
extern bool SD_Card_Present; // Flag for SD Card Presence
extern WingSailType WingSail;
extern configValuesType Configuration;
extern MPU9250 WingSailAngleSensor;

static const int MaxParameterIndex = 67;

void SendMessage(int CommandPort, TelMessageType msg)
{
	char FloatString[16];

	switch (msg)
	{
	case TelMessageType::LNA:
		(*Serials[CommandPort]).print(F("LNA,"));
		(*Serials[CommandPort]).print(NavData.CTE);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.DTW);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.BTW);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.CDA);

		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.Currentloc.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.COG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.SOG_mps);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.HDG);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LAT:
		(*Serials[CommandPort]).print(F("LAT,"));
		(*Serials[CommandPort]).print(NavData.HDG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print((int)myIMU.pitch);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print((int)myIMU.roll);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.ROLL_Avg);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.VMG);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LPO:
		(*Serials[CommandPort]).print(F("LPO,"));
		(*Serials[CommandPort]).print(PowerSensorV);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(PowerSensorI);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(0); // Wingsail Voltage
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(0); // Wingsail Current
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LWP:
		(*Serials[CommandPort]).print(F("LWP,"));
		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.prev_WP.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(NavData.next_WP.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.MaxCTE);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LMI:
		(*Serials[CommandPort]).print(F("LMI,"));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.mission_size);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].cmd);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].duration);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].SteerAWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[StateValues.mission_index].TrimTabAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LWI:
		(*Serials[CommandPort]).print(F("LWI,"));
		(*Serials[CommandPort]).print(NavData.AWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.TWD);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.TWS);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.Angle);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.TrimTabAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LSV:
		(*Serials[CommandPort]).print(F("LSV,"));
		(*Serials[CommandPort]).print(SteeringServoOutput);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(WingSail.Servo_microseconds);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LVS:
		(*Serials[CommandPort]).print(F("LVS,"));
		(*Serials[CommandPort]).print(StateValues.CommandState);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CommandStateToString(StateValues.CommandState));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LPF:
		(*Serials[CommandPort]).print(F("LPF,"));
		(*Serials[CommandPort]).print(NavData.AWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.SOG_mps);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.VMG);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(Configuration.TrimTabDefaultAngle);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MCC:
		(*Serials[CommandPort]).print(F("MSG,Mission Command List Cleared."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MIG:
		(*Serials[CommandPort]).print(F("MSG,Mission Command Index is: "));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::MIS:
		(*Serials[CommandPort]).print(F("MSG,Mission Command Index Set to:"));
		(*Serials[CommandPort]).print(StateValues.mission_index);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::HLG:
		(*Serials[CommandPort]).print("HOM,");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lat) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lng) / 10000000UL, 10, 5, FloatString));
		(*Serials[CommandPort]).print(",");
		StateValues.home_is_set ? (*Serials[CommandPort]).print("Set:true") : (*Serials[CommandPort]).print("Set:false");
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.DTH);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(NavData.BTH);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	//case TelMessageType::HLS:
	//	(*Serials[CommandPort]).print(F("HLS: Set Home Location."));
	//	(*Serials[CommandPort]).println();

	//	(*Serials[CommandPort]).print("HOM,");
	//	(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lat) / 10000000UL, 10, 5, FloatString));
	//	(*Serials[CommandPort]).print(",");
	//	(*Serials[CommandPort]).print(dtostrf(float(StateValues.home.lng) / 10000000UL, 10, 5, FloatString));
	//	(*Serials[CommandPort]).print(",");
	//	StateValues.home_is_set ? (*Serials[CommandPort]).print("Set:true") : (*Serials[CommandPort]).print("Set:false");
	//	(*Serials[CommandPort]).println();
	//	MessageArray[msg] = 0;
	//	break;

	case TelMessageType::VER:
		(*Serials[CommandPort]).print("VER,");
		(*Serials[CommandPort]).print(Version);
		(*Serials[CommandPort]).print(" ");
		(*Serials[CommandPort]).println(VersionDate);
		MessageArray[msg] = 0;
		break;

	case TelMessageType::TMG:
		(*Serials[CommandPort]).print(F("TIM,20"));
		(*Serials[CommandPort]).print(CurrentLocalTime.year, DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CurrentLocalTime.month, DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CurrentLocalTime.dayOfMonth, DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CurrentLocalTime.hour, DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CurrentLocalTime.minute, DEC);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(CurrentLocalTime.second, DEC);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LOG:
		(*Serials[CommandPort]).print(F("MSG,SD Card Logging Level set to: "));
		(*Serials[CommandPort]).print(Configuration.SDCardLoggingMask);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::LCD:
		(*Serials[CommandPort]).print(F("MSG,Display Screen View set to: "));
		(*Serials[CommandPort]).print(Configuration.DisplayScreenView);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::EQG:
		(*Serials[CommandPort]).print(F("MSG:EQUIP"));
		(*Serials[CommandPort]).print(F(",SD Card: "));
		if (SD_Card_Present)
			(*Serials[CommandPort]).print(F("OK"));
		else
			(*Serials[CommandPort]).print(F("Fail"));

		(*Serials[CommandPort]).print(",WingsailAngle: ");
		if (WingSailAngleSensor.Initialsed)
			(*Serials[CommandPort]).print(F("OK"));
		else
			(*Serials[CommandPort]).print(F("Fail"));

		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::CCS:
		(*Serials[CommandPort]).print(F("MSG,Compass Calibration Save."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::CCG:
		(*Serials[CommandPort]).print(F("MSG,Compass Status:"));
		(*Serials[CommandPort]).print(myIMU.Algorithm_Status);
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::WC0:
		(*Serials[CommandPort]).print(F("MSG,Wingsail Angle Sensor Calibration: Completed & Saved."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::WC1:
		(*Serials[CommandPort]).print(F("MSG,Wingsail Angle Sensor Calibration: Begin."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::SCS:
	case TelMessageType::SCG:
		(*Serials[CommandPort]).print(F("MSG,Command State set to: "));
		ShowCommandState(CommandPort, StateValues.CommandState);
		// if the command includes a parameter, then print that as well
		switch (StateValues.CommandState)
		{
		case vcsSteerWindCourse:
			(*Serials[CommandPort]).print(",");
			(*Serials[CommandPort]).print(StateValues.SteerWindAngle);
			break;

		case vcsSteerMagneticCourse:
			(*Serials[CommandPort]).print(",");
			(*Serials[CommandPort]).print(StateValues.SteerCompassBearing);
			break;
		default:;
		}
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

	case TelMessageType::DBG:
		(*Serials[CommandPort]).print(F("MSG,10 minute Debug."));
		(*Serials[CommandPort]).println();
		MessageArray[msg] = 0;
		break;

// Message Array
	case TelMessageType::MCP:
		SendMissionStep(CommandPort, MissionValues.mission_size - MessageArray[msg] );
		MessageArray[msg]--;
		break;

	case TelMessageType::PRL:
		ListParameter(CommandPort, MaxParameterIndex - MessageArray[msg] + 1);
		MessageArray[msg]--;
		break;

	default:;
	}

}

void ProcessQueue(int SerialPortNumber)
{
// find the first non-zero flag in the message array, send the corresponding message, and clear the flag
	int msg = 0;
	while ((MessageArray[msg] == 0) && (msg < EndMarker))
	{
		msg++;
	};

	if (msg < EndMarker)
	{
		// if we are here, its because we found something before hitting the endMarker
		SendMessage(SerialPortNumber, (TelMessageType) msg);
	};
}

void QueueMessage(TelMessageType msg)
{
	switch (msg) 
	{
	    case TelMessageType::MCP:
			MessageArray[msg] = MissionValues.mission_size;
			break;

		case TelMessageType::PRL:
			MessageArray[msg] = MaxParameterIndex;
			break;

		default:
			MessageArray[msg] = 1;	
	}
}


void SendMissionStep(int CommandPort, int i)
{
	char FloatFormatString[16];
	(*Serials[CommandPort]).print("MCP,");
	MissionCommandType mc = MissionValues.MissionList[i].cmd;

	switch (mc)
	{
	case ctGotoWaypoint:
		(*Serials[CommandPort]).print(ctGotoWaypoint);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	case ctLoiter:
		(*Serials[CommandPort]).print(ctLoiter);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).println();
		break;

	case ctLoiterUntil:
		(*Serials[CommandPort]).print(ctLoiterUntil);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lat) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(dtostrf(float(MissionValues.MissionList[i].waypoint.lng) / 10000000UL, 10, 5, FloatFormatString));
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).println();
		break;

	case ctReturnToHome:
		(*Serials[CommandPort]).print(ctReturnToHome);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].boundary);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	case ctSteerWindCourse:
		(*Serials[CommandPort]).print(ctSteerWindCourse);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].SteerAWA);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].TrimTabAngle);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].duration);
		(*Serials[CommandPort]).print(",");
		(*Serials[CommandPort]).print(MissionValues.MissionList[i].controlMask);
		(*Serials[CommandPort]).println();
		break;

	default:
		(*Serials[CommandPort]).print(F("Unknown command"));
	}

}