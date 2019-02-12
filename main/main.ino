#define DEBUG Serial 
#define DEBUG_BAUDRATE 115200

// uncomment/comment debug options
#define DEBUG_NO2
#define DEBUG_NCO
#define DEBUG_OPC
#define DEBUG_CLOCK

// uncomment which platform to build it for
// Note this should be set in the configuration manager
//#define TEENSY_35
//#define ARDUINO_PRO_MINI
#define ARDUINOR3

#if defined (TEENSY_35)
#include "opcn2.h"
#elif defined (ARDUINOR3) || defined(ARDUINO_PRO_MINI)
#include "opcn2m.h"
#endif

#include <SPI.h>

// Clock library
#include <SparkFunDS3234RTC.h>

// plug the LCD module to the I2C bus
#if defined(TEENSY_35) || defined(ARDUINOR3)
#define LCB_AVAIL
#if defined(LCB_AVAIL)
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3,POSITIVE);
#endif

// NOTE: Arduino Pro mini won't have an LCD component
#endif

#if defined(TEENSY_35)
#include <SD.h>
// SD variables
int sd_log_counter;
bool sd_is_file_open = false;
String pollutionLogStr;
File PollutionLog;
String sd_file_name = "teensy35data.txt";

// chip select for opcn2 is set to pin 10 on Teensy 3.5
int OPC_CS = 10;

#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)
#include <SdFat.h>
SdFat sd;
SdFile PollutionLog;
char sd_file_name[] = "arduinoR3data.csv";

String pollutionLogStr;
bool sd_is_file_open = false;
// chip select for opcn2 is set to pin 8 on Arduino Uno and pro mini
const int OPC_CS = 8;
// chip select for arduino
const int SD_CS = 9;

// DS13074 chip select pin 
const int CLOCK_CS = 10;
#endif



// OPCN2 Variables
#if defined(TEENSY_35)

OPCN2 alpha(OPC_CS);
HistogramData hist;
ConfigVars vars;

#elif defined (ARDUINOR3) ||  defined (ARDUINO_PRO_MINI)

OPCN2 alpha(OPC_CS);
HistogramData hist;
ConfigVars vars;

#endif

#if defined (TEENSY_35) 
// No2 - Variables
#define CO_PIN A9
#define COx_PIN A8

// NCo - Variables
#define NO2_PIN A7
#define NO2x_PIN A6
#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)
// No2 - Variables		
#define CO_PIN A0		
#define COx_PIN A1		
						
// NCo - Variables		
#define NO2_PIN A2		
#define NO2x_PIN A3		
#endif

struct
{
	int CO;
	int COx;
}Co;

struct
{
	int NO2;
	int NO2x;
} No2;

struct 
{
	// Read the time:
	int s;
	int m;
	int h;
	// Read the day/date:
	int dy;
	int da;
	int mo;
	int yr;

} clock;

// start from here to read the code
void setup() {
	DEBUG.begin(DEBUG_BAUDRATE);
#if defined(LCB_AVAIL) 
	lcd_setup();
#endif

	opc_setup();
	clock_setup();
	sd_setup();

	delay(3000);
}

void loop() {
	// TODO: introduce multithreading
	// TODO: port each process into a library? maybe
	No2_loop();
	Co_loop();
	opc_printHistogram();
	clock_loop();
	sd_loop();

#if defined(LCB_AVAIL)
	lcd_loop();
#endif
}
// end read here

int sd_setup()
{
	DEBUG.println("SD Setup");
#if defined(TEENSY_35)
	if (!SD.begin(BUILTIN_SDCARD))
	{
		DEBUG.println("SD: Failed to setup...");
	}
	else {
		DEBUG.println("SD: Set Up success");
	}
#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)

	while(!sd.begin(SD_CS, SPI_HALF_SPEED))
	{
		DEBUG.println("SD CARD: Failed to set up SD Card");
		delay(1000);
	}

	delay(100);
#endif

	pollutionLogStr = "y ,m, d, day,h,m,s, CO, COx, NO2, NO2x, PM1, PM2.5, PM10,";
	sd_is_file_open = false;
}

void sd_loop() 
{
	if (!sd_is_file_open) {
		// open the file
#if defined(TEENSY_35)
		PollutionLog = SD.open(sd_file_name, FILE_WRITE);
		if (PollutionLog) {
			sd_is_file_open = true;
		}
#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)
		if (PollutionLog.open(sd_file_name, O_WRITE | O_CREAT | O_APPEND)) {
			sd_is_file_open = true;
		}
#endif
	} 

	if (sd_is_file_open) {
		DEBUG.println("writting to SD card: " + pollutionLogStr);
		PollutionLog.println(pollutionLogStr);
		PollutionLog.close();
		// print to the serial port too:
		Serial.println(pollutionLogStr);
		sd_is_file_open = false;
	// if the file isn't open, pop up an error:
	} else {
		Serial.println("error opening PollutionLog.csv");
	}

	// log data 
	PollutionLog.println(pollutionLogStr);
 
	pollutionLogStr = String(clock.dy) + 
					"," + String(clock.yr) + 
					"," + String(clock.mo) +
					"," + String(clock.da) + 
					"," + String(clock.h)  +
					"," + String(clock.m)  +
					"," +  String(clock.s) +
					"," + String(Co.CO)	   +
					"," + String(Co.COx)   +
					"," + String(No2.NO2)  +
					"," + String(No2.NO2x) +
					"," + String(hist.pm1) +
					"," + String(hist.pm25)+
					"," + String(hist.pm10);

	// To make sure we log 100 times before closing the file
	//sd_log_counter++;
	//if (sd_log_counter > 10) {
	//	PollutionLog.close();
	//	sd_log_counter = 0;
	//	sd_file_status = false;
	//}

	//DEBUG.println(pollutionLogStr);

}

void clock_setup()
{
	DEBUG.println("Clock Set up");
	rtc.begin(CLOCK_CS);
	//rtc.autoTime();
	// Or you can use the rtc.setTime(s, m, h, day, date, month, year)
	// function to explicitly set the time:
	// e.g. 7:32:16 | Monday October 31, 2016:
	//rtc.setTime(00, 40, 18, 7, 9, 02, 19);  // Uncomment to manually set time
}

void clock_loop()
{
	rtc.update();
	// Read the time:
	clock.s = rtc.second();
	clock.m = rtc.minute();
	clock.h = rtc.hour();
	// Read the day/date:
	clock.dy = rtc.day();
	clock.da = rtc.date();
	clock.mo = rtc.month();
	clock.yr = rtc.year();

#if defined(DEBUG_CLOCK)
	DEBUG.print(String(clock.s) + ":" + String(clock.m) + ":" + String(clock.h)) + ":";
	DEBUG.println(":" + String(clock.da) + ":" + String(clock.mo) +":" +String(clock.yr));
#endif
}

void opc_setup()
{
#if defined(TEENSY_35) 
	DEBUG.println("Testing OPC-N2 v" + String(alpha.firm_ver.major) + "." + String(alpha.firm_ver.minor));

	// Read and print the configuration variables
	vars = alpha.read_configuration_variables();
	DEBUG.println("\nConfiguration Variables");
	DEBUG.print("\tGSC:\t"); DEBUG.println(vars.gsc);
	DEBUG.print("\tSFR:\t"); DEBUG.println(vars.sfr);
	DEBUG.print("\tLaser DAC:\t"); DEBUG.println(vars.laser_dac);
	DEBUG.print("\tFan DAC:\t"); DEBUG.println(vars.fan_dac);
	DEBUG.print("\tToF-SFR:\t"); DEBUG.println(vars.tof_sfr);

	// Turn on the OPC
	alpha.on();
	delay(1000);
#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)

	pinMode(OPC_CS, OUTPUT);
	DEBUG.println("Testing OPC-N2 v" + String(alpha.firm_ver.major) + "." + String(alpha.firm_ver.minor));
	delay(1000);
	// Read and print the configuration variables
	vars = alpha.read_configuration_variables();
	Serial.println("\nConfiguration Variables");
	Serial.print("\tGSC:\t"); Serial.println(vars.gsc);
	Serial.print("\tSFR:\t"); Serial.println(vars.sfr);
	Serial.print("\tLaser DAC:\t"); Serial.println(vars.laser_dac);
	Serial.print("\tFan DAC:\t"); Serial.println(vars.fan_dac);
	Serial.print("\tToF-SFR:\t"); Serial.println(vars.tof_sfr);
	
	// wait for it to turn on
	while (!alpha.on()) {
		DEBUG.println("Turning ON OPC");
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Turning ON OPC");
	}
#endif
}

// displays different size of particles
void opc_printHistogram()
{
#if defined (TEENSY_35)
	//delay(500);
	hist = alpha.read_histogram();

#if defined(DEBUG_OPC)
	// Print out the histogram data
	DEBUG.print("\nSampling Period:\t"); Serial.println(hist.period);
	DEBUG.print("PM1: "); Serial.println(hist.pm1);
	DEBUG.print("PM2.5: "); Serial.println(hist.pm25);
	DEBUG.print("PM10: "); Serial.println(hist.pm10);
#endif
#elif defined(ARDUINO_PRO_MINI) || defined(ARDUINOR3)

	delay(500);
	hist = alpha.read_histogram();
	// Print out the histogram data
	DEBUG.print("\nSampling Period:\t"); Serial.println(hist.period);
	DEBUG.print("PM1: "); Serial.println(hist.pm1);
	DEBUG.print("PM2.5: "); Serial.println(hist.pm25);
	DEBUG.print("PM10: "); Serial.println(hist.pm10);

#endif
}

// reads No2 levels
void No2_loop() 
{
	No2.NO2 = (int)((analogRead(NO2_PIN)*(5000 / 1024) - 348) / 0.434f);
	No2.NO2x = (int)((analogRead(NO2x_PIN)*(5000 / 1024) - 348) / 0.434f);
#if defined(DEBUG_NO2) 
	DEBUG.println("No2.NO2:" + String(No2.NO2));
	DEBUG.println("No2.NO2x: " + String(No2.NO2x));
#endif

}
// reads Co levels
void Co_loop() 
{
	Co.CO = (int)((analogRead(CO_PIN)*(5000 / 1024) - 348) / 0.434f);
	Co.COx = (int)((analogRead(COx_PIN)*(5000 / 1024) - 348) / 0.434f);
#if defined(DEBUG_NCO)
	DEBUG.println("Co.CO: " + String(Co.CO));
	DEBUG.println("Co.COx: " + String(Co.COx));
#endif
}

#if defined(LCB_AVAIL) 
void lcd_setup() 
{
	lcd.begin(20,4);
	lcd.clear();
	lcd.print("Monitor Setup");
	DEBUG.println("LCD Setup Success");
	delay(1000);
}

void lcd_loop() 
{
	lcd.display();

	lcd.setCursor(0, 0);
	lcd.print("Pollution Monitor: ");

	lcd.setCursor(0, 1);
	lcd.print("CO = " + String(Co.CO));

	lcd.setCursor(0, 2);
	lcd.print("NO2 = " + String(No2.NO2));
	//lcd.setCursor(0, 1);
	//lcd.print("SO2=");
	//lcd.print(SO2);

	//lcd.clear();
	//if (A0 + A2 + A5 < 1500)
	//{
	//	lcd.print("LOW Pollution");
	//}
	//
	//if (A0 + A2 + A5 > 1500)
	//{
	//	lcd.print("HIGH Pollution");
	//}
	//
	//delay(3000);
	lcd.clear();
	lcd.noDisplay();
}
#endif