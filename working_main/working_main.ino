int CO;
int COx;
int NO2;
int NO2x;
//int T;
//int H;

//sds
#include <SdFat.h>
SdFat sd;
SdFile file;
//Adafruit_ADS1115 ads1(0x48);
//Adafruit_ADS1115 ads2(0x49);
//struct ts t;
char newfile[] = "data4.csv";
const int CS = 10; //

				   //const int N = 10;
				   //unsigned int sensorDataCO[N];
				   //int i = 0;
				   //int value;

				   //pm sensor
#include <Arduino.h>
#include <SoftwareSerial.h>
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM01Value = 0;          //define PM1.0 value of the air detector module
int PM2_5Value = 0;         //define PM2.5 value of the air detector module
int PM10Value = 0;         //define PM10 value of the air detector module
SoftwareSerial PMSerial(7, 8); // RX, TX

							   //clock
#include <SPI.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SparkFunDS3234RTC.h>
#define DS13074_CS_PIN 9
#define INTERRUPT_PIN 2
unsigned char flag;


// Read the time:
int s = rtc.second();
int m = rtc.minute();
int h = rtc.hour();
// Read the day/date:
int dy = rtc.day();
int da = rtc.date();
int mo = rtc.month();
int yr = rtc.year();

//SHT temp
//#include <Wire.h>
#include <SHTSensor.h>
SHTSensor sht;


void setup()
{
	Serial.begin(9600);
	Serial.println("Start");

	String dataString = "";
	dataString += ("y ,m, d, day,h,m,s, CO, COx, NO2, NO2x, PM1, PM2.5, PM10, temperature, humidity");


	//clock
	rtc.begin(DS13074_CS_PIN);
	//rtc.autoTime();
	// Or you can use the rtc.setTime(s, m, h, day, date, month, year)
	// function to explicitly set the time:
	// e.g. 7:32:16 | Monday October 31, 2016:
	rtc.setTime(00, 25, 19, 5, 17, 05, 18);  // Uncomment to manually set time
	Serial.print("clock - ");
	Serial.println(rtc.dayStr());

	//pm
	PMSerial.begin(9600);
	PMSerial.setTimeout(1500);


	//sd
	Serial.println("sd");
	while (!sd.begin(CS, SPI_HALF_SPEED)) {
	}
	file.open(newfile, O_WRITE | O_CREAT | O_APPEND);
	file.close();
	delay(100);

	//pm sensor
	PMSerial.begin(9600);
	PMSerial.setTimeout(1500);

	//SHT temp
	sht.init();
	Wire.begin();

	Serial.println("y ,m, d, day,h,m,s, CO, COx, NO2, NO2x, PM1, PM2.5, PM10, temperature, humidity");
}

// Sleep Call
void sleepNow() {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
	sleep_enable();          // enables the sleep bit in the mcucr register
	attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function  
	sleep_mode();            // here the device is actually put to sleep!!  
							 // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
	sleep_disable();         // first thing after waking from sleep: disable sleep...  
	detachInterrupt(0);      // disables interrupt 0 on pin 2 so the wakeUpNow code will not be executed during normal running time.
}
void wakeUpNow() {
	// call when wake. Don't use timer or any other built in function
	flag = 1;
}

void loop()
{
	rtc.update(); // Update RTC data

				  //Serial.println (ReadTimeDate());
				  //Serial.println (time1());
	CO = (analogRead(A0)); //*(5000/1024)-348)/0.434;
	COx = (analogRead(A1)); //*(5000/1024)-348)/0.434;
	NO2 = (analogRead(A2)); //*(5000/1024)-348)/0.434;
	NO2x = (analogRead(A3)); //*(5000/1024)-348)/0.434;

							 //alarm
	rtc.enableAlarmInterrupt();
	//rtc.setAlarm1(60); // Alarm 1 triggers every second
	rtc.setAlarm2(); // Alarm 2 triggers every minute
	sleepNow();     // sleep function called here  

					//sensorDataCO[i] = CO;
					//sensorDataNO2[i] = NO2;
					//sensorDataSO2[i++] = SO2;
					//if (i >=N) i = 0;

					//unsigned long accum = 0;

					//for(int j = 0; j < N; j++)

	{
		//accumCO = accum + sensorDataCO[j];
		//accumNO = accum + sensorDataNO2[j];
	}


	// pm sensor
	if (PMSerial.find(0x42)) {
		PMSerial.readBytes(buf, LENG);
		if (buf[0] == 0x4d) {
			if (checkValue(buf, LENG)) {
				PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
				PM2_5Value = transmitPM2_5(buf);//count PM2.5 value of the air detector module
				PM10Value = transmitPM10(buf); //count PM10 value of the air detector module 
			}
		}
	}

	if (rtc.alarm1())
		//Serial.println("ALARM 1 is triggered!");
		if (rtc.alarm2())
			Serial.println("ALARM 2 is triggered!");

	//sd

	String dataString = "";

	dataString += rtc.year();
	dataString += ",";
	dataString += rtc.month();
	dataString += ",";
	dataString += rtc.date();
	dataString += ",";
	dataString += rtc.day();
	dataString += ",";
	dataString += rtc.hour();
	dataString += ",";
	dataString += rtc.minute();
	dataString += ",";
	dataString += rtc.second();

	dataString += (", ");
	dataString += (CO);
	dataString += (", ");
	dataString += (COx);
	dataString += (", ");
	dataString += (NO2);
	dataString += (", ");
	dataString += (NO2x);
	dataString += (", ");
	dataString += (PM01Value);
	dataString += (", ");
	dataString += (PM2_5Value);
	dataString += (", ");
	dataString += (PM10Value);
	dataString += (", ");
	dataString += (sht.getTemperature());
	dataString += (", ");
	dataString += (sht.getHumidity());

	// while (!sd.begin(CS,SPI_HALF_SPEED)) {
	//     
	// } // initialises SD card again - for when sd card is removed for data

	file.open(newfile, O_WRITE | O_APPEND); //Opens the file
	delay(5);
	file.println(dataString); //prints data string to the file
	delay(5);
	file.close(); //closes the file
	delay(5);


	Serial.println(dataString);

	delay(6000);
}

char checkValue(unsigned char *thebuf, char leng)
{
	char receiveflag = 0;
	int receiveSum = 0;

	for (int i = 0; i<(leng - 2); i++) {
		receiveSum = receiveSum + thebuf[i];
	}
	receiveSum = receiveSum + 0x42;

	if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1]))  //check the serial data 
	{
		receiveSum = 0;
		receiveflag = 1;
	}
	return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
	int PM01Val;
	PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
	return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
	int PM2_5Val;
	PM2_5Val = ((thebuf[5] << 8) + thebuf[6]);//count PM2.5 value of the air detector module
	return PM2_5Val;
}

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
	int PM10Val;
	PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module  
	return PM10Val;
}
