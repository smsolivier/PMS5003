#pragma once 

#include <Arduino.h>

class PMS5003 {
public:
  PMS5003(Stream &stream) : _serial(stream) { }
  struct Data {
	uint16_t framelen; 
	uint16_t pm_st[3]; 
	uint16_t pm_en[3]; 
	uint16_t hist[6]; 
	uint16_t unused; 
	uint16_t checksum; 
	bool valid = false; 
	byte mask = 0; 
  }; 
  enum ValidMasks {
	HAVE_START1 = 1, 
	HAVE_START2 = 2, 
	HAVE_LENGTH = 4, 
	HAVE_CHECKSUM = 8
  }; 
  enum OperatingMode {
	ACTIVE,
	PASSIVE
  }; 
  enum Status {
	ASLEEP,
	WOKE,
	REQUESTING
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
