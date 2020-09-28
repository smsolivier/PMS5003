# PMS5003 
The PMS5003 is the air quality sensor used by PurpleAir in their air quality monitors. It measures the amount of dust in the air and the data it collects can be used to compute the air quality index (AQI) used to assess how safe the air is to breathe. This library was designed in the process of making a battery powered AQI sensor. It is then crucial to power the sensor down in between sensors to increase battery life. 

This library wraps a `HardwareSerial` or `SoftwareSerial` stream corresponding to a PMS5003's TX and RX pins. The library facilitates changing the sensor's operating modes (active vs. passive), waking/sleeping the sensor, and reading the data from the sensor. This library was written since other PMS5003 libraries lacked reliability when `delay`'s were used. 

# Fix for unreliable sensor readings 
Across many Arduino forums there appears to be trouble with reliably reading the PMS5003's output when not continuously reading the sensor. This issue is caused by the serial buffer overflowing and overwriting data. The easiest fix is to make sure to continuously empty the serial buffer so that an overflow never occurs. However, for low power applications it is necessary to power down the sensor between data measurements. Additionally, Plantower recommends waiting 30 seconds after returning power to the PMS5003 before taking measurements to give time for the fan to turn on. Given that the PMS5003 is outputting at 9600 BAUD = 9600 bits per second, the 64 byte buffer will fill in 53 milliseconds. Thus, operating the sensor for more than 53 milliseconds without reading from the serial buffer will cause an overflow. Reliability was increased (at least for my Arduino Nano) by 
1. draining the serial buffer completely before taking a measurement. This is achieved by running 
```cpp
for (int i=0; i<Serial.available(); i++) Serial.read(); 
``` 
before each measurement. This clears any corrupted data from the buffer. 

2. increase the default serial buffer size by one byte. I noticed that despite the buffer being nominally 64 bytes, `Serial.available()` had a maximum value of 63 bytes (using both `HardwareSerial` and `SoftwareSerial`). This means that a full buffer can hold one uncorrupted PMS5003 32 byte message and one corrupted PMS5003 message with one byte overwritten by the uncorrupted message. By increasing the buffer size to 65 bytes in `SoftwareSerial.h`/`HardwareSerial.h`, `Serial.available()` now returns 64 bytes when the buffer is full. 

The combination of these two methods led to very reliable measurements. Unfortunately, the buffer size in `SoftwareSerial` and `HardwareSerial` are not configurable from an external library. The macro corresponding to the receive buffer size must be altered in the core Arduino library files (on Linux `/usr/share/arduino/hardware`). 

# `SoftwareSerial` Example 
```cpp
#include <SoftwareSerial.h> 
#include "PMS5003.hpp" 

SoftwareSerial pms_serial(2,3); // PMS5003 TX connected to pin 2, RX to pin 3 
PMS5003 pms(pms_serial); 
PMS5003::Data data; 

void setup() {
	Serial.begin(9600); 
	pms_serial.begin(9600); 
	Serial.print("starting PMS5003 with SoftwareSerial buffer size = "); 
	Serial.println(_SS_MAX_RX_BUFF); 

	pms.SetPassive(); // only send data when requested, helps reduce buffer overflow 
	pms.SetDrainBuffer(true); // drain the serial buffer before reading a measurement 

	delay(1000); // give the fan time to startup 
}

void loop() {
	// wake the PMS5003 
	// wait 30 seconds for the fan to turn on 
	// request the data from the sensor 
	// read the message and store the output in the data struct 
	// sleep the sensor 
	// tries holds the number of read attempts. 
	// data.valid == true means the data was correctly read 
	// data.mask holds debugging info and will indicate where the read failed
	// data.mask == 15 indicates 
		// the first byte of the message is 0x42 (the PMS5003 start of message byte)
		// the second byte of the message matches the PMS5003's second indicating byte 
		// the message is 32 bytes as expected 
		// the message passes the checksum test 
	size_t tries = pms.ForcedRead(data, 30000); 

	// print the "standard" PM1.0, PM2.5, and PM10.0 values read from the sensor 
	String s = String("1.0: ") + String(data.pms.pm_st[0]) + String(", 2.5: ") 
		+ String(data.pms.pm_st[1]) + String(", 10.0: ") + String(data.pms.pm_st[2]); 
	if (!data.valid) {
		Serial.print("data invalid. validity mask = "); 
		Serial.println(data.mask); 
	} else Serial.println(s); 

	delay(60000); // wait a minute before taking the next measurement 
}
