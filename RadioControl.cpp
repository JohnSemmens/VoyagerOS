// Radio Control and Servo Handler
// 
// V1.0 12/10/2016 John Semmens
// V1.1 27/04/2018 added debouncing to the RC switch position value

#include "RadioControl.h"
#include <Servo.h>
#include "configValues.h"

extern RC_IN_Type RC_IN;
extern configValuesType Configuration;

Servo Servo_Channel[4]; // Array size is set to the quantity of servo output channels.

volatile uint16_t RC_Ch0InShared;
volatile uint16_t RC_Ch1InShared;
volatile uint16_t RC_Ch2InShared;
volatile uint16_t RC_Ch3InShared;
//volatile uint16_t RC_Ch4InShared;
volatile uint8_t bUpdateFlagsShared;

// Assign RC channel input pins on Arduino Mega Board
#define RC_Ch0_IN_PIN	2
#define RC_Ch1_IN_PIN	3
#define RC_Ch2_IN_PIN	19
#define RC_Ch3_IN_PIN	18
//#define RC_Ch4_IN_PIN	13

// Assign Servo channel output pins on Arduino Mega Board. These are normal output pins; not specially assigned.
#define Servo_Ch0_OUT_PIN	8
#define Servo_Ch1_OUT_PIN	9
#define Servo_Ch2_OUT_PIN	10
#define Servo_Ch3_OUT_PIN	11
//#define Servo_Ch4_OUT_PIN	12

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR.
uint32_t RC_Ch0_IN_Start; 
uint32_t RC_Ch1_IN_Start;
uint32_t RC_Ch2_IN_Start;
uint32_t RC_Ch3_IN_Start;
//uint32_t RC_Ch4_IN_Start;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define RC_Ch0_IN_FLAG 1
#define RC_Ch1_IN_FLAG 2
#define RC_Ch2_IN_FLAG 4
#define RC_Ch3_IN_FLAG 8
//#define RC_Ch4_IN_FLAG 16

#define RC_PulseWidth_Min 1000
#define RC_PulseWidth_Max 2000

// Interrupt Service Routines
void isrCh0_IN()
{
	if (digitalRead(RC_Ch0_IN_PIN) == HIGH)
		RC_Ch0_IN_Start = micros();
	else
	{
		RC_Ch0InShared = (uint16_t)(micros() - RC_Ch0_IN_Start);
		bUpdateFlagsShared |= RC_Ch0_IN_FLAG;
	}
}

void isrCh1_IN()
{
	if (digitalRead(RC_Ch1_IN_PIN) == HIGH)
		RC_Ch1_IN_Start = micros();
	else
	{
		RC_Ch1InShared = (uint16_t)(micros() - RC_Ch1_IN_Start);
		bUpdateFlagsShared |= RC_Ch1_IN_FLAG;
	}
}

void isrCh2_IN()
{
	if (digitalRead(RC_Ch2_IN_PIN) == HIGH)
		RC_Ch2_IN_Start = micros();
	else
	{
		RC_Ch2InShared = (uint16_t)(micros() - RC_Ch2_IN_Start);
		bUpdateFlagsShared |= RC_Ch2_IN_FLAG;
	}
}

void isrCh3_IN()
{
	if (digitalRead(RC_Ch3_IN_PIN) == HIGH)
		RC_Ch3_IN_Start = micros();
	else
	{
		RC_Ch3InShared = (uint16_t)(micros() - RC_Ch3_IN_Start);
		bUpdateFlagsShared |= RC_Ch3_IN_FLAG;
	}
}

void RC_Init(void)
{
	// V1.0 13/10/2016 John Semmens

	pinMode(RC_Ch0_IN_PIN, INPUT_PULLUP);
	pinMode(RC_Ch1_IN_PIN, INPUT_PULLUP);
	pinMode(RC_Ch2_IN_PIN, INPUT_PULLUP);
	pinMode(RC_Ch3_IN_PIN, INPUT_PULLUP);
	//pinMode(RC_Ch4_IN_PIN, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(RC_Ch0_IN_PIN), isrCh0_IN, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RC_Ch1_IN_PIN), isrCh1_IN, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RC_Ch2_IN_PIN), isrCh2_IN, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RC_Ch3_IN_PIN), isrCh3_IN, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(RC_Ch4_IN_PIN), isrCh4_IN, CHANGE);
}

void Servos_Init(void)
{
	// initalise the array of Servo objects by attaching to pins.
	Servo_Channel[0].attach(Servo_Ch0_OUT_PIN);
	Servo_Channel[1].attach(Servo_Ch1_OUT_PIN);
	Servo_Channel[2].attach(Servo_Ch2_OUT_PIN);
	Servo_Channel[3].attach(Servo_Ch3_OUT_PIN);

	// set each channel to the failsafe value
	Servo_Channel[0].write(Configuration.RC_FailSafe_Ch0);
	Servo_Channel[1].write(Configuration.RC_FailSafe_Ch1);
	Servo_Channel[2].write(Configuration.RC_FailSafe_Ch2);
	Servo_Channel[3].write(Configuration.RC_FailSafe_Ch3);
}

void RC_Read(void)
{
	// V1.0 13/10/2016 John Semmens

	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off while we make local copies of the shared variables
		RC_IN.Channel[0] = RC_Ch0InShared;
		RC_IN.Channel[1] = RC_Ch1InShared;
		RC_IN.Channel[2] = RC_Ch2InShared;
		RC_IN.Channel[3] = RC_Ch3InShared;
	//	RC_IN.Channel[4] = RC_Ch4InShared;
		bUpdateFlagsShared = 0;
		interrupts();

		RC_IN.LastRCSignalTime = millis();
		RC_IN.SignalValid = true;
	}

	// test for valid RC signal and set to Failsafe values if time has been exceeded.
	if ((millis() - RC_IN.LastRCSignalTime) > (Configuration.RC_FailSafe_Time * 1000))
	{
		RC_IN.SignalValid = false;
		RC_IN.Channel[0] = Configuration.RC_FailSafe_Ch0;
		RC_IN.Channel[1] = Configuration.RC_FailSafe_Ch1;
		RC_IN.Channel[2] = Configuration.RC_FailSafe_Ch2;
		RC_IN.Channel[3] = Configuration.RC_FailSafe_Ch3;
		//RC_IN.Channel[4] = Configuration.RC_FailSafe_Ch4;
	}

	// Interpret the RC Command Channel value to yield a "Switch" Position from the continuously variable control
	RC_Command_Switch();
};

void RC_Command_Switch(void)
{
	// Interpret the RC Command Channel value to yield a "Switch" Position from the continuously variable control
	// The position is determined by the thresholds setup in Config.
	// V1.0 17/10/2016 John Semmens
	// V1.1 27/4/2018 added de-bouncing to prevent a change on only one event
	// V1.2 26/11/2018 closed gap between switch positions, where there was a one microsecond gap. This was causing inadvertant "0. Idle" responses

	int Switch_Position; // current value
	static int Switch_Position1; // previous value
	static int Switch_Position2; //  previous, previous value

	uint16_t PulseLength = RC_IN.Channel[0];
	Switch_Position = 0;

	// command Switch Position 1
	if (PulseLength < Configuration.RC_IN_Command_Threhold1)
	{
		Switch_Position = 1;
	}

	// command Switch Position 2
	if (PulseLength >= Configuration.RC_IN_Command_Threhold1 && PulseLength < Configuration.RC_IN_Command_Threhold2)
	{
		Switch_Position = 2;
	}

	// command Switch Position 3
	if (PulseLength >= Configuration.RC_IN_Command_Threhold2 && PulseLength < Configuration.RC_IN_Command_Threhold3)
	{
		Switch_Position = 3;
	}

	// command Switch Position 4
	if (PulseLength >= Configuration.RC_IN_Command_Threhold3 && PulseLength < Configuration.RC_IN_Command_Threhold4)
	{
		Switch_Position = 4;
	}

	// command Switch Position 5
	if (PulseLength >= Configuration.RC_IN_Command_Threhold4)
	{
		Switch_Position = 5;
	}

	// set the failsafe value to 9. i.e. when the RC Signal is not present.
	if (RC_IN.SignalValid == false)
	{
		Switch_Position = 9;
	}

	// only change state if there's three consequtive passes with the same signal value
	if ((Switch_Position == Switch_Position1) && (Switch_Position1 == Switch_Position2))
	{
		RC_IN.RC_Command_Switch_Position = Switch_Position;
	}

	// shuffle the previous values down.
	Switch_Position2 = Switch_Position1;
	Switch_Position1 = Switch_Position;	
}

void Servo_Out(int Channel,int PulseWidth)
{
	// output a value to a servo.
	// V1.0 13/10/2016 John Semmens

	// check that the request is within resonable bounds before writing to the servo.
	if (( RC_PulseWidth_Min <= PulseWidth ) && ( PulseWidth <= RC_PulseWidth_Max )) {
		Servo_Channel[Channel].writeMicroseconds(PulseWidth);
	}
};
