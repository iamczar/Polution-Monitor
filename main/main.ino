#define DEBUG Serial 
#define DEBUG_BAUDRATE 115200

// uncomment/comment debug options
#define DEBUG_NO2
#define DEBUG_SO
#define DEBUG_CO
#define DEBUG_O3
#define DEBUG_OPC
#define DEBUG_CLOCK


// OPC SENSOR
#include "opcn2.h"
// chip select for opcn2 is set to pin 10 on Teensy 3.5
int OPC_CS = 31;

#include <SPI.h>

// Clock library
#include <SparkFunDS3234RTC.h>

// plug the LCD module to the I2C bus
#include "LiquidCrystal_I2C.h"

// Instantiate an LCD
LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3,POSITIVE);

#include <SD.h>
// SD variables
int sd_log_counter;
bool is_sd_file_open = false;
String pollutionLogStr;
File PollutionLog;
const char sd_file_name[] = "T35data.csv";

// DS13074 chip select pin 
const int CLOCK_CS = 6;

// OPCN2 Variables

OPCN2 alpha(OPC_CS);
HistogramData hist;
ConfigVars vars;

// CO - Variables
#define CO_PIN A16
#define COx_PIN A17

// NO2 - Variables
#define NO2_PIN A14
#define NO2x_PIN A15

// O3 - Variables
#define O3_PIN A18
#define O3x_PIN A19

// So - Variables - Not used yet
//#define SO_PIN A3
//#define SOx_PIN A2

// GAS LED traffic lights
#define gas_red_led_pin 17
#define gas_yellow_led_pin 16
#define gas_green_led_pin 15

// PM LED traffic lights
#define pm_red_led_pin 28
#define pm_yellow_led_pin 29
#define pm_green_led_pin 30



struct
{
	int CO;
	int COx;
	uint8_t light;
}Co;

struct
{
	int NO2;
	int NO2x;
	uint8_t light; // pollution severity indicator
} No2;

struct
{
  int SO;
  int SOx;
  uint8_t light;
}So;

struct
{
  int O3;
  int O3x;
  uint8_t light;
}O3;

struct {
	float light;
	String light_str;
}gas;

struct {
	float light;
	String light_str;
}Pm;

struct {
	float light;
	String light_str;
}pm1;

struct {
	float light;
	String light_str;
}pm25;

struct {
	float light;
	String light_str;
}pm10;


struct
{
	// Read the time:
	uint8_t s;
	uint8_t m;
	uint8_t h;
	// Read the day/date:
	uint8_t dy;
	uint8_t da;
	uint8_t mo;
	uint8_t yr;
} clock;

enum NO_LEVELS 
{
	NO_LOW_LVL  = 100,
	NO_HIGH_LVL = 200
};

enum CO_LEVELS
{
	CO_LOW_LVL = 100,
	CO_HIGH_LVL = 200
};

enum O3_LEVELS
{
	O3_LOW_LVL = 100,
	O3_HIGH_LVL = 200
};

enum SO_LEVELS
{
	SO_LOW_LVL = 100,
	SO_HIGH_LVL = 200
};

enum LED 
{
	RED = 0,
	YELLOW = 1,
	GREEN = 2
};

// start from here to read the code
void setup() {
	DEBUG.begin(DEBUG_BAUDRATE);

	led_setup();

	lcd_setup();

	opc_setup();

	clock_setup();

	sd_setup();

	lcd.clear();

	lcd.print("Waiting...");

	delay(3000);
}

void loop() {
	// TODO: introduce multithreading
	// TODO: port each process into a library? maybe
	No2_loop();
	
	//So_loop();
	
	Co_loop();
	
	O3_loop();
	
	opc_printHistogram();
	
	clock_loop();
	
	sd_loop();
	
	lcd_loop();

}
// end read here

void led_setup() 
{
	pinMode(gas_red_led_pin,OUTPUT);
	pinMode(gas_yellow_led_pin, OUTPUT);
	pinMode(gas_green_led_pin, OUTPUT);

	pinMode(pm_red_led_pin, OUTPUT);
	pinMode(pm_yellow_led_pin, OUTPUT);
	pinMode(pm_green_led_pin, OUTPUT);
}

void gas_led_turn_on(int led)
{
	LED local_led = (LED)led;
	switch (local_led)
	{
		case RED:
			digitalWrite(gas_red_led_pin,HIGH);
			break;
		case YELLOW:
			digitalWrite(gas_yellow_led_pin,HIGH);
			break;
		case GREEN:
			digitalWrite(gas_green_led_pin,HIGH);
			break;
	}
}

void gas_led_turn_off(int led)
{
	LED local_led = (LED)led;
	switch (local_led)
	{
	case RED:
		digitalWrite(gas_red_led_pin, LOW);
		break;
	case YELLOW:
		digitalWrite(gas_yellow_led_pin, LOW);
		break;
	case GREEN:
		digitalWrite(gas_green_led_pin, LOW);
		break;
	}
}

void pm_led_turn_on(int led)
{
	LED local_led = (LED)led;
	switch (local_led)
	{
	case RED:
		digitalWrite(gas_red_led_pin, HIGH);
		break;
	case YELLOW:
		digitalWrite(gas_yellow_led_pin, HIGH);
		break;
	case GREEN:
		digitalWrite(gas_green_led_pin, HIGH);
		break;
	}
}

void pm_led_turn_off(int led)
{
	LED local_led = (LED)led;
	switch (local_led)
	{
	case RED:
		digitalWrite(pm_red_led_pin, LOW);
		break;
	case YELLOW:
		digitalWrite(pm_yellow_led_pin, LOW);
		break;
	case GREEN:
		digitalWrite(pm_green_led_pin, LOW);
		break;
	}
}



void sd_setup()
{
	DEBUG.println("SD Setup");
	if (!SD.begin(BUILTIN_SDCARD)) {
		DEBUG.println("SD: Failed to setup...");
	} else {
		DEBUG.println("SD: Set Up success");
	}

	pollutionLogStr = "y ,m, d, day,h,m,s, CO, COx, NO2, NO2x, O3, O3x PM1, PM2.5, PM10,";
	is_sd_file_open = false;
}

void sd_loop()  
{

	if (!is_sd_file_open) {
		// open the file
		PollutionLog = SD.open(sd_file_name, FILE_WRITE);
		if (PollutionLog) {
			is_sd_file_open = true;
		}
	} 

	if (is_sd_file_open) {
		DEBUG.println("writting to SD card: " + pollutionLogStr);
		PollutionLog.println(pollutionLogStr);
		PollutionLog.close();
		// print to the serial port too:
		Serial.println(pollutionLogStr);
		is_sd_file_open = false;

	// if the file isn't open, pop up an error:
	} else {
		Serial.println("error opening " + String(sd_file_name));
	}

	// write polution log into a file 
	PollutionLog.println(pollutionLogStr);
 
	// build the data into a string
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
					"," + String(O3.O3)    +
					"," + String(O3.O3x)   +
					"," + String(hist.pm1) +
					"," + String(hist.pm25)+
					"," + String(hist.pm10);

    //temperature/hum TODO

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
	lcd.clear();
	lcd.print("Setting up CLK");

	DEBUG.println("Clock Set up");
	rtc.begin(CLOCK_CS);
	rtc.autoTime();
	rtc.set24Hour(true);

	// Or you can use the rtc.setTime(s, m, h, day, date, month, year)
	// function to explicitly set the time:
	// e.g. 7:32:16 | Monday October 31, 2016:
	//rtc.setTime(00, 40, 18, 7, 9, 02, 19);  // Uncomment to manually set time
	delay(1000);

	lcd.clear();
	lcd.print("Clock Setup");
}

void clock_loop()
{
	rtc.update();

	// Read the time:
	clock.h = rtc.hour();
	clock.m = rtc.minute();
	clock.s = rtc.second();

	// Read the day/date:
	clock.dy = rtc.day();
	clock.da = rtc.date();
	clock.mo = rtc.month();
	clock.yr = rtc.year();

#if defined(DEBUG_CLOCK)
	DEBUG.print("h:" + String(clock.h) + " min:" + String(clock.m) + " sec:" + String(clock.s)) + ":";
	DEBUG.println("date:" + String(clock.da) + " :mon" + String(clock.mo) +" yr:" +String(clock.yr) + "//" +String(clock.dy) );
#endif
}

void opc_setup()
{
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
	lcd.clear();
	lcd.print("Setting up OPC");

	// wait for it to turn on
	int opc_turn_attempt = 0;
	bool alpha_status = false;

	while (!alpha_status) {
		alpha_status = alpha.on();
		alpha.off();
		opc_turn_attempt++;
		// wait half a second
		delay(500);
		DEBUG.println("Turning ON OPC:" + String(opc_turn_attempt) +": "+ String(alpha_status));
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Turning ON OPC :" + String(opc_turn_attempt));
	}

	lcd.clear();
	lcd.print("OPC ON");

}

// displays different size of particles
void opc_printHistogram()
{
	hist = alpha.read_histogram();

#if defined(DEBUG_OPC)
	// Print out the histogram data
	DEBUG.print("\nSampling Period:\t"); Serial.println(hist.period);
	DEBUG.print("PM1: "); Serial.println(hist.pm1);
	DEBUG.print("PM2.5: "); Serial.println(hist.pm25);
	DEBUG.print("PM10: "); Serial.println(hist.pm10);
#endif

}

// reads nitrogen dioxide levels
void No2_loop() 
{
	No2.NO2 = (int)((((analogRead(NO2_PIN)*(5000.0f / 1024.0f) - 238.0f) / 0.259f)/1.9125f)/1000.0f); //ug
	No2.NO2x = (int)((((analogRead(NO2x_PIN)*(5000.0f / 1024.0f) - 235.0f) / 0.259f)/1.9125f)/1000.0f);
#if defined(DEBUG_NO2) 
	DEBUG.println("No2.NO2:" + String(No2.NO2));
	DEBUG.println("No2.NO2x: " + String(No2.NO2x));
#endif

}
// reads carbon monoxide levels
void Co_loop() 
{
	Co.CO = (int)(((analogRead(CO_PIN)*(5000.0f / 1024.0f) - 348.0f) / 0.434f)/1.1642f); //mg
	Co.COx = (int)(((analogRead(COx_PIN)*(5000.0f / 1024.0f) - 348.0f) / 0.434f)/1.1642f);
#if defined(DEBUG_CO)
	DEBUG.println("Co.CO: " + String(Co.CO));
	DEBUG.println("Co.COx: " + String(Co.COx));
#endif
}

// reads Ozone levels
void O3_loop() 
{
  O3.O3 = (int)((((analogRead(O3_PIN)*(5000.0f / 1024.0f) - 217.0f) / 0.342f)/1.9957f)/1000.0f); //ug
  O3.O3x = (int)((((analogRead(O3x_PIN)*(5000.0f / 1024.0f) - 216.0f) / 0.342f)/1.9957f)/1000.0f);
#if defined(DEBUG_SO)
  DEBUG.println("O3.O3: " + String(O3.O3));
  DEBUG.println("O3.O3x: " + String(O3.O3x));
#endif
}

// reads sulfur dioxide levels
/*
void So_loop() 
{
  So.SO = (int)(((analogRead(SO_PIN)*(5000 / 1024) - 348) / 0.43)/2.6609); //mg
  So.SOx = (int)(((analogRead(SOx_PIN)*(5000 / 1024) - 348) / 0.434)/2.6609);
#if defined(DEBUG_SO)
  DEBUG.println("So.SO: " + String(So.SO));
  DEBUG.println("So.SOx: " + String(So.SOx));
#endif
}
*/

void trafficlight_loop() {

	if (No2.NO2 > NO_LOW_LVL) {
		No2.light = 0;
	}

	if (No2.NO2 > NO_LOW_LVL && No2.NO2 < NO_HIGH_LVL) {
		No2.light = 1;
	}

	if (No2.NO2 > NO_HIGH_LVL) {
		No2.light = 2;
	}

	if (Co.CO > 3) {
		Co.light = 0;
	}

	if (Co.CO > 3 && Co.CO < 10) {
		Co.light = 1;
	}

	if (Co.CO > 10) {
		Co.light = 2;
	}

	if (O3.O3 > 60) {
		So.light = 0;
	}

	if (O3.O3 > 60 && So.SO < 120) {
		O3.light = 1;
	}
	 
	if (O3.O3 > 120) {
		O3.light = 2;
	}

	if (So.SO > 50) {
		So.light = 0;
	}
	if (So.SO > 50 && So.SO < 350) {
		So.light = 1;
	}

	if (So.SO > 350) {
		So.light = 2;
	}

	gas.light = (O3.light + Co.light + No2.light) / 3.0f;

	if (gas.light == 0) {
		gas.light_str = "LOW";
		
		//LED green
		gas_led_turn_on(GREEN);
	}

	if (gas.light == 1) {
		gas.light_str = "MEDIUM";
		
		//LED amber
		gas_led_turn_on(YELLOW);
	}

	if (gas.light == 2) {
		gas.light_str = "HIGH";
		
		//LED red
		gas_led_turn_on(RED);
	}

	if (hist.pm1 > 3) {
		pm1.light = 0;
	}

	if (hist.pm1 > 3 && hist.pm1 < 10) {
		pm1.light = 1;
	}

	if (hist.pm1 > 10) {
		pm1.light = 2;
	}

	if (hist.pm25 > 10) {
		pm25.light = 0;
	}

	if (hist.pm25 > 10 && hist.pm25 < 25) {
		pm25.light = 1;
	}

	if(hist.pm25 > 25) {
		pm25.light = 2;
	}

	if (hist.pm10 > 25) {
		pm10.light = 0;
	}

	if (hist.pm10 > 25 && hist.pm10 < 50) {
		pm10.light = 1;
	}

	if (hist.pm10 > 50) {
		pm10.light = 2;
	}

	Pm.light = (pm10.light + pm25.light + pm1.light) / 3;

	if (Pm.light == 0) {
		Pm.light_str = "LOW";
		//LED green
		pm_led_turn_on(GREEN);
	}

	if (Pm.light == 1.0f) {
		Pm.light_str = "MEDIUM";
		//LED amber
		pm_led_turn_on(GREEN);
	}

	if (Pm.light == 2) {
		Pm.light_str = "HIGH";
		//LED red 
		pm_led_turn_on(GREEN);
	}
}

void lcd_setup() 
{
	lcd.begin(20,4);
	lcd.clear();
	lcd.print("Monitor Setup");
	DEBUG.println("LCD Setup Success");
}

void lcd_loop() 
{
	lcd.display();

//should be in LCD setup: TODO ?
//	lcd.setCursor(0, 0);
	//lcd.print("Pollution Monitor: ");
 
	//if there is a problem TODO  //LCD.print("Pollution Monitor call engineer: ");  

	//lcd.setCursor(0, 0);
	//lcd.print("Gas pollution: "); lcd.print(gas.light); //lcd.print(gas.light); // can this work yet?
	lcd.setCursor(0, 1);
	lcd.print("CO = " + String(Co.CO)); lcd.print("mg/m^3");

	lcd.setCursor(0, 2);
	lcd.print("NO2 = " + String(No2.NO2)); lcd.print("ug/m^3");

	lcd.setCursor(0, 3);
	lcd.print("O3 = " + String(O3.O3)); lcd.print("ug/m^3");
	//lcd.print("O3 = ");  lcd.print(O3.O3); lcd.print("ug/m^3"); try this if above does not work...


	//lcd.setCursor(0, 3);
	//lcd.print("SO2 = " + String(So.SO)); lcd.print("mg/m^3");

	lcd.clear();
	lcd.setCursor(0, 0);
	//lcd.print("Particle count: "); lcd.print(Pm.light); // 
	lcd.setCursor(0, 1);
	lcd.print("PM1: "); lcd.print(hist.pm1); lcd.print("ug/m3 : ");

    lcd.setCursor(0, 2);
    lcd.print("PM2.5: "); lcd.print(hist.pm25); lcd.print("ug/m3 : ");

    lcd.setCursor(0, 3);
    lcd.print("PM10: "); lcd.print(hist.pm10); lcd.print("ug/m3 : ");

    delay(2000);

    lcd.clear();
  
	lcd.clear();
	lcd.noDisplay();
}

//temp hum sensor TODO
//button turn on screen TODO
// traffic light RGB ordered TODO
// to do microphone when button pressed

