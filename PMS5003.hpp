#pragma once 

#include <Arduino.h>

// wraps a serial stream such as HardwareSerial or SoftwareSerial corresponding to a PMS5003 sensor 
// facilitates toggling operating modes and reading the data 
// it is recommended to increase the serial buffer size for more reliable reading 
// the default buffer size is set to 64 bytes but it appears to only actually be able to store 63 bytes
// I think the circular buffer implemented in SoftwareSerial may be buggy when the 
// buffer overflows (from not reading the PMS5003 often enough) 
// increasing the buffer size to 65 bytes and draining the buffer before reading a 
// message makes reading the serial stream 100% reliable (for my arduino nano at least) 
class PMS5003 {
public:
	// constructor: provide a Stream that has RX and TX from PMS5003 
	PMS5003(Stream &stream) : _serial(stream) { }
	// data structure holding numeric output from sensor. 26 bytes total 
	struct Data {
		uint16_t pm_st[3]; // "standard" particulate matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters 
		uint16_t pm_en[3]; // "environmental" particular matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters
		uint16_t hist[6]; // number of particles detected in 0.1 L beyond 0.3 um, 0.5 um, 1.0 um, 2.5 um, 5.0 um, and 10 um 
		byte mask = 0; // stores info corresonding to ValidMasks (useful for debugging which part of the message was wrong) 
		bool valid = false; // data is trustworthy 
	}; 
	// bitwise comparison enum 
	// used to create a error code for Data::mask 
	enum ValidMasks {
		HAVE_AVAILABLE = 1, // serial has 32 bytes available 
		HAVE_START1 = 2, // message's first byte is 0x42 
		HAVE_START2 = 4, // message's second byte is 0x4d 
		HAVE_LENGTH = 8, // message is 32 bytes long 
		HAVE_CHECKSUM = 16, // message passes the checksum test 
		HAVE_VALID = 31 // message has correct starting bytes, length, and checksum passed 
	}; 
	enum OperatingMode {
		ACTIVE, // PMS5003 continuously streams data 
		PASSIVE // PMS5003 is on but only sends data if requested 
	}; 
	enum Status {
		ASLEEP, // PMS5003 is asleep 
		WOKE, // PMS5003 is active but not sending data if in passive mode 
		REQUESTING // PMS5003 is active and sending data (if in passive mode) 
	}; 
	// drain serial buffer after call to Wake() 
	void SetDrainBuffer(bool drain) { _drain = drain; }
	// how long to search for the start character 
	void SetSeekTimeout(unsigned long timeout) { _seek_timeout = timeout; }
	// how long to call Read() before giving up 
	void SetBlockingTimeout(unsigned long timeout) { _blocking_timeout = timeout; }
	// how long to wait for the fan to startup after calling Wake() 
	void SetStartupDelay(unsigned long startup_delay) { _startup_delay = startup_delay; }

	// drain the serial input buffer 
	void DrainBuffer(); 

	// read the serial buffer for the AQI data and store in data 
	void Read(Data &data); 
	// wraps Read in case of failures. returns number of calls to Read
	size_t BlockingRead(Data &data); 
	// wakes sensor if necessary, calls BlockingRead, and returns to sleep if it was asleep at beginnging of call 
	// returns number of calls to Read 
	size_t ForcedRead(Data &data); 
	// wakes sensor if necessary, reads for avg_time, averages the data, and sleeps if it was asleep before 
	// returns maximum number of calls to Read in the avg_time time period 
	size_t AveragedRead(Data &data, unsigned long avg_time=10000); 

	// tell PMS5003 to sleep (lower power state) 
	void Sleep(); 
	// tell PMS5003 to wake from sleep 
	void Wake(); 
	// tell PMS5003 to go into passive operating mode 
	// turns fan on but doesn't send data until a request for the data is made 
	// helps not overfill serial buffer when just waiting for the fan to start 
	void SetPassive(); 
	// tell PMS5003 to go into active operating mode 
	// continuously streams data to serial buffer 
	void SetActive(); 
	// tell PMS5003 to start sending data 
	// only necessary for passive mode 
	void RequestData(); 
private:
	// 32 B message sent by the PMS5003 
	struct SensorOutput {
		uint16_t framelen; // size of data in bytes 
		uint16_t pm_st[3]; // "standard" particulate matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters 
		uint16_t pm_en[3]; // "environmental" particular matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters
		uint16_t hist[6]; // number of particles detected in 0.1 L beyond 0.3 um, 0.5 um, 1.0 um, 2.5 um, 5.0 um, and 10 um 
		uint16_t unused; // blank 
		uint16_t checksum; // bitwise sum to check against 
	}; 
	// convert from SensorOutput struct to Data struct 
	void ExtractData(const SensorOutput &idata, Data &data); 

	// set all values of Data struct to zero 
	void ZeroDataStructure(Data &data); 
	// do d1 += d2 on numeric values only. carry mask and valid forward 
	void AddDataStructures(Data &d1, Data &d2); 
	// divide by an integer 
	void DivideDataStructure(Data &data, unsigned long N); 

	Stream &_serial; // PMS5003 serial 
	byte _mode = ACTIVE; // active v passive mode 
	byte _status = WOKE; // operating status
	unsigned long _seek_timeout = 2000; // how long to seek for the 0x42 start byte 
	unsigned long _blocking_timeout = 10000; 
	bool _drain = true; // control whether to drain the serial buffer after Wake() 
	unsigned long _startup_delay = 30000; // how long to delay after calling Wake() 
}; 
