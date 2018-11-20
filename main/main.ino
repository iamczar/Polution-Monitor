#define DEBUG Serial 
#define DEBUG_BAUDRATE 115200
#define DEBUG_NO2
#define DEBUG_NCO

#include "opcn2.h"
#include <SPI.h>

// OPC-N2 variables
// chip select is set to pin 10 on Teensy 3.5
#define CS = 10;
OPCN2 alpha(10);
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
}

void loop() {
	No2_loop();
	Co_loop();
	opc_printHistogram();
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
	No2.NO2 = analogRead(NO2_PIN);
	No2.NO2x = analogRead(NO2x_PIN);
#if defined(DEBUG_NO2) 
	DEBUG.println("No2.NO2:" + String(No2.NO2));
	DEBUG.println("No2.NO2x: " + String(No2.NO2x));
#endif

}

// reads Co levels
void Co_loop() 
{
	Co.CO = analogRead(CO_PIN);
	Co.COx = analogRead(COx_PIN);
#if defined(DEBUG_NCO)
	DEBUG.println("Co.CO: " + String(Co.CO));
	DEBUG.println("Co.COx: " + String(Co.COx));
#endif
}
