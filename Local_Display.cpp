// 
// Local Display Handler
// This is generic local display handler.
// It passes display logging page requests to the available options for local display.
// These local display options are: 0.91" OLED and 20x4 LCD
// V1.0 12/12/2017 John Semmens

#include "Local_Display.h"
//#include "LCD_Logging.h"
#include "OLED_Logging.h"

void Local_Display_Logging(char LoggingLevel)
{
	OLED_Logging(LoggingLevel);
	//LCD_Logging(LoggingLevel);
}

