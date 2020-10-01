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

	// convert to nicer data structure 
	ExtractData(_idata, data); 

	// set valid mask if have all the requirements for a good measurement 
	data.valid = data.mask & HAVE_VALID; 
}

size_t PMS5003::BlockingRead(Data &data) {
	// do nothing if asleep 
	if (_status == ASLEEP) {
		data.mask = 0; 
		data.valid = false; 
		return 0; 
	}
	// request data if necessary 
	if (_mode == PASSIVE and _status != REQUESTING) RequestData();

	unsigned long prev = millis(); 
	size_t tries = 0; 
	while (millis() - prev < _blocking_timeout) { // loop for at most timeout milliseconds 
		Read(data); // attempt a read 
		tries++; 
		if (data.valid) break; // exit if valid data 
	}
	return tries; 
}

size_t PMS5003::ForcedRead(Data &data) {
	byte prev_status = _status; 
	if (_status == ASLEEP) Wake(); // wake and wait if asleep 
	size_t tries = BlockingRead(data);  
	if (prev_status == ASLEEP) Sleep(); // return to sleep if started asleep 
	return tries; 
}

size_t PMS5003::AveragedRead(Data &data, unsigned long avg_time) {
	byte prev_status = _status; 
	if (_status == ASLEEP) Wake(); // wake and wait if asleep 

	size_t max_tries = 0; 
	size_t N = 0; // number of calls to read 
	Data tmp_data; 
	ZeroDataStructure(data); 
	unsigned long prev = millis(); 
	while (millis() - prev < avg_time) {
		size_t tries = BlockingRead(tmp_data); 
		if (tries > max_tries) max_tries = tries; 
		AddDataStructures(data, tmp_data); 
		N++; 
	}
	DivideDataStructure(data, N); 

	if (prev_status == ASLEEP) Sleep(); // return to sleep if started asleep 
	return max_tries; 
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
	delay(_startup_delay); 
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

void PMS5003::ZeroDataStructure(Data &data) {
	for (int i=0; i<3; i++) {
		data.pm_st[i] = 0; 
		data.pm_en[i] = 0; 
	}
	for (int i=0; i<6; i++) {
		data.hist[i] = 0; 
	}
	data.mask = HAVE_VALID; 
	data.valid = true; 
}

void PMS5003::AddDataStructures(Data &d1, Data &d2) {
	for (int i=0; i<3; i++) {
		d1.pm_st[i] += d2.pm_st[i]; 
		d1.pm_en[i] += d2.pm_en[i]; 
	}
	for (int i=0; i<6; i++) {
		d1.hist[i] += d2.hist[i]; 
	}
	d1.mask = min(d1.mask, d2.mask); 
	d1.valid = d1.valid and d2.valid; 
}

void PMS5003::DivideDataStructure(Data &data, unsigned long N) {
	for (int i=0; i<3; i++) {
		data.pm_st[i] = (uint16_t)(roundf((float)data.pm_st[i]/N)); 
		data.pm_en[i] = (uint16_t)(roundf((float)data.pm_en[i]/N)); 
	}
	for (int i=0; i<6; i++) {
		data.hist[i] = (uint16_t)(roundf((float)data.hist[i]/N)); 
	}
}