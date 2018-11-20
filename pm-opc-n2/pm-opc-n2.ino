#define DEBUG Serial 
#define DEBUG_BAUDRATE 115200

#include "opcn2.h"
#include <SPI.h>

// chip select is set to pin 10 on Teensy 3.5
#define CS 10
OPCN2 alpha(CS);
HistogramData hist;
ConfigVars vars;

void setup() {
	DEBUG.begin(DEBUG_BAUDRATE);
	opc_setup();
}

void loop() {
	printHistogram();
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

void printHistogram() 
{
	delay(500);
	hist = alpha.read_histogram();

	// Print out the histogram data
	DEBUG.print("\nSampling Period:\t"); Serial.println(hist.period);
	DEBUG.print("PM1: "); Serial.println(hist.pm1);
	DEBUG.print("PM2.5: "); Serial.println(hist.pm25);
	DEBUG.print("PM10: "); Serial.println(hist.pm10);
}
