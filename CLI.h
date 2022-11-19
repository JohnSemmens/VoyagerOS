// CLI.h
// Command Processor - for Voyager
// Interpret commands received through the serial port
// V1.0 22/12/2015

// include something like this in setup(): 	Serial.begin(9600);

#ifndef _CLI_h
#define _CLI_h

/*
	==================================
	Serial Commands
	==================================

	ech: Echo the command and parameters
		ech,param1,param2,param3,param4,param5,param6
	mcs: Mission Command Set
	scs: Set Command State
	scg: Get Command State
	mce: Mission Command execute now
	mis: Mission Command Index Set
	mig: Mission Command Index Get
	mcl: Mission Command List
	mcp: Mission Command List for plotting
	mcc:  Mission Command List - Clear All
	lcg: Get Current Location
	swc: Steer Course Relative to Wind
	stc : Steer True Course
	idl: Set the vessel to an idle state
	man: Set control to manual, via RC
	hls, Set Home Location, relative to current postion
		 hls,range metres, bearing degrees
	hlg, Get Home Location
		hlg
	hlc, Clear the Home Location
		hlc
	ver, Get Software Version
		ver
	tmg, Get Current Time
		tmg
	tms, Set Current Time
		tms,yy,mm,dd,hh,mm,ss 
	log, Set Logging Level on the SD Card
		log,level
	tel, Set Telemetry Reporting Level.
		tel,level 
	sav, Save Config/mission/state to EEPROM
		sav,c/m/s
	lcs: Set Current Location. override GPS and simulate location
		lcs,lat deg,lon deg
	wc1: Wingsail Calibration On
	wc0: Wingsail Calibration Off

	ccs: Compass Calibration Save

	//cal, Calibrate Magnetic Compass mode  
	//	cal,y/n
	//ccg, Get Compass Calibration Values

	//ccs, Set Compass Calibration Values
	//	ccs, 6 paramters
	//gcg, Get Gyro Calibration 

	//gcs, Get Gyro Calibration
	//	gcs, 1 param

	prg, Parameter Get
		prg, parameter number
	prl, Parameter List All
		no paramters
	prs, Parameter Set 
		prs, parameter number, value
	
	dcs,  Parameter Set

	dcg,  Parameter Get

	dsp, Display Message on the LCD/OLED
		dsp,line1,line2

	vmg, Get Voltage Measurements

    LoRa commands
	LNA - Approach:	CTE, DTW, BTW, CDA, LAT, LON, SOG, COG, HDG
	LAT - Attitude: HDG, Pitch, Roll, Roll_avg
	LPO - Power:	Voyager Power ,Wingsail Power
	LWP - Waypoints:  Prev WP, Next WP
	LMI - current Mission Step,Max CTE, MI, MS
	LWI - AWA/TWD, AWS/TWS,WA/TTA
	LSV - Rudder us, Trim Tab us
	LVS - GetVessel State - Vessel State

	not done, because these already exist as other commands
	LMA - GetMission array
	LGT - GetTime
	LGH - Get Home Location


*/


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AP_Math.h"
#include "CommandState_Processor.h"


void CLI_Process_Message(int CommandPort);

void CLI_Processor(int CommandPort, String Command);
void CLI_Processor(int CommandPort);
void ListParameter(int CommandPort, int ParameterIndex);
void ShowCommandState(int CommandPort, VesselCommandStateType cs);


#endif

