#include "PMS5003.hpp"

void PMS5003::Read(Data &data) {
	// reset mask 
	data.mask = 0; 
	
	// drain buffer 
	if (_drain) for (int i=0; i<_serial.available(); i++) _serial.read(); 

	// search for start character 
	unsigned long prev = millis(); 
	while (millis() - prev < _seek_timeout) {
		if (_serial.peek() == 0x42) break; 
		_serial.read(); 
	}

	// read 32 bytes 
	size_t len = _serial.readBytes(_buf, sizeof(_idata)); 
	if (_buf[0] == 0x42) data.mask |= HAVE_START1; 
	if (_buf[1] == 0x4d) data.mask |= HAVE_START2; 
	if (len == 32) data.mask |= HAVE_LENGTH; 

	// convert to single precision 
	// skip first two bytes 
	for (int i=0; i<15; i++) {
		_buf16[i] = _buf[2 + 2*i + 1]; 
		_buf16[i] += _buf[2 + 2*i] << 8; 
	}

	// copy to first 30 bytes of data structure 
	memcpy(&_idata, _buf16, 30); 

	// compute checksum 
	uint16_t sum = 0; 
	for (int i=0; i<30; i++) {
	  sum += _buf[i];     
	}
	if (_idata.checksum == sum) data.mask |= HAVE_CHECKSUM; 

	ExtractData(_idata, data); 
}

size_t PMS5003::BlockingRead(Data &data, unsigned long timeout) {
	// do nothing if asleep 
	if (_status == ASLEEP) {
		data.mask = SENSOR_ASLEEP_ERR; 
		return 0; 
	}
	// request data if necessary 
	if (_mode == PASSIVE and _status != REQUESTING) RequestData();

	unsigned long prev = millis(); 
	size_t tries = 0; 
	while (millis() - prev < timeout) { // loop for at most timeout milliseconds 
		Read(data); // attempt a read 
		tries++; 
		if (data.mask & HAVE_VALID) break; // exit if valid data 
	}
	if (tries==0) {
		data.mask = SENSOR_TIMEOUT_ERR; 
	}
	return tries; 
}

void PMS5003::AveragedRead(Data &data, unsigned long avg_time, unsigned long timeout) {
	if (_status == ASLEEP) {
		data.mask = SENSOR_ASLEEP_ERR; 
		return; 
	}
	if (_mode == PASSIVE and _status != REQUESTING) RequestData(); 

	unsigned long prev = millis(); 
	while (millis() - prev < avg_time) {
		BlockingRead(data, timeout); 
	}
}

size_t PMS5003::ForcedRead(Data &data, unsigned long startup_delay, unsigned long timeout) {
	// wake and wait if asleep 
	if (_status == ASLEEP) {
		Wake(); 
		delay(startup_delay); // wait for fan to startup 
	}
	size_t tries = BlockingRead(data, timeout);  
	Sleep(); 
	return tries; 
}

// see https://usermanual.wiki/Pdf/plantowerpms5003manualannotated.626592918/html for commands 
void PMS5003::Sleep() {
	uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };  
	_serial.write(command, sizeof(command)); 
	_status = ASLEEP; 
}

void PMS5003::Wake() {
	uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 }; 
	_serial.write(command, sizeof(command)); 
	_status = WOKE; 
}

void PMS5003::SetPassive() {
	uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 }; 
	_serial.write(command, sizeof(command)); 
	_mode = PASSIVE; 
}

void PMS5003::SetActive() {
	uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 }; 
	_serial.write(command, sizeof(command)); 
	_mode = ACTIVE; 
}

void PMS5003::RequestData() {
	uint8_t command[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 }; 
	_serial.write(command, sizeof(command)); 
	_status = REQUESTING; 
}

void PMS5003::ExtractData(const SensorOutput &idata, Data &data) {
	memcpy(&data, ((byte*)&idata) + 2, 24); // skip first 2 bytes and copy 24 bytes 
}