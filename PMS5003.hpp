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
  PMS5003(Stream &stream) : _serial(stream) { }
  struct Data {
	uint16_t framelen; // size of data in bytes 
	uint16_t pm_st[3]; // "standard" particulate matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters 
	uint16_t pm_en[3]; // "environmental" particular matter in micro grams / liter for 1.0, 2.5, and 10 micron diameters
	uint16_t hist[6]; // number of particles detected in 0.1 L beyond 0.3 um, 0.5 um, 1.0 um, 2.5 um, 5.0 um, and 10 um 
	uint16_t unused; // blank 
	uint16_t checksum; // bitwise sum to check against 
	bool valid = false; // this data was read correctly (matches starting bytes, length, checksum)
	byte mask = 0; // stores info corresonding to ValidMasks (useful for debugging which part of the message was wrong) 
  }; 
  enum ValidMasks {
	HAVE_START1 = 1, // message's first byte is 0x42 
	HAVE_START2 = 2, // message's second byte is 0x4d 
	HAVE_LENGTH = 4, // message is 32 bytes 
	HAVE_CHECKSUM = 8 // message passes the checksum test 
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
  // turn on/off draining the serial buffer before taking a measurement 
  // improves reliability when not taking measurements continuously 
  void SetDrainBuffer(bool drain=true) { _drain = drain; }
  // how long to search for the start character 
  void SetSeekTimeout(size_t timeout) { _seek_timeout = timeout; }
  // read the serial buffer for the AQI data and store in data 
  void Read(Data &data); 
  // wraps Read in case of failures 
  size_t BlockingRead(Data &data, size_t timeout=5000); 
  // wraps BlockingRead to include waking, waiting for fan to start, and sleeping the sensor
  size_t ForcedRead(Data &data, size_t startup_delay=30000, size_t timeout=5000); 
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
  Stream &_serial; // PMS5003 serial 
  byte _buf[32]; // store a PMS5003 32 byte message 
  uint16_t _buf16[15]; // store the data converted to half precision ints (without the first two bytes)
  byte _mode = ACTIVE; // active v passive mode 
  byte _status = WOKE; // operating status
  size_t _seek_timeout = 2000; // how long to seek for the 0x42 start byte 
  bool _drain = true; // control whether to drain the serial buffer before taking a measurement 
}; 
