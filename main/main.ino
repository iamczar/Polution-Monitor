#define DEBUG Serial 
#define DEBUG_BAUDRATE 115200

// uncomment applicable options
#define DEBUG_NO2
#define DEBUG_NCO
#define LCB_AVAIL

// choose which microcontroller you want
#define TEENSY_SPECIFIC
//#define ARDUINO_PRO_MINI

#include "opcn2.h"
#include <SPI.h>

#if defined(LCB_AVAIL)
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3,POSITIVE);
#endif

// OPC-N2 variables
#if defined(TEENSY_SPECIFIC)
// chip select is set to pin 10 on Teensy 3.5
int OPC_CS = 10;
#elif defined(ARDUINO_PRO_MINI)
//int OPC_CS = 10;
#elif defined(ARDUINOR3)
//int OPC_CS = 10;
#endif

OPCN2 alpha(OPC_CS);
HistogramData hist;
ConfigVars vars;

// No2 - Variables
#define CO_PIN A9
#define COx_PIN A8
struct
{
	int CO;
	int COx;
}Co;

// NCo - Variables
#define NO2_PIN A7
#define NO2x_PIN A6
struct
{
	int NO2;
	int NO2x;
} No2;

void setup() {
	DEBUG.begin(DEBUG_BAUDRATE);
	opc_setup();
	lcd_setup();
}

void loop() {
	No2_loop();
	Co_loop();
	opc_printHistogram();
	lcd_loop();
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
	alpha.on();
	delay(1000);
}

// displays different size of particles
void opc_printHistogram()
{
	delay(500);
	hist = alpha.read_histogram();

	// Print out the histogram data
	DEBUG.print("\nSampling Period:\t"); Serial.println(hist.period);
	DEBUG.print("PM1: "); Serial.println(hist.pm1);
	DEBUG.print("PM2.5: "); Serial.println(hist.pm25);
	DEBUG.print("PM10: "); Serial.println(hist.pm10);
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

void lcd_setup() 
{
	lcd.begin(20,4);
	lcd.clear();
	lcd.print("Monitor Setup");
	delay(5000);
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
	delay(3000);


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
