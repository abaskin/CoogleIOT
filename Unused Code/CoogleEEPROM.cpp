/*
  +----------------------------------------------------------------------+
  | CoogleIOT for ESP8266                                                |
  +----------------------------------------------------------------------+
  | Copyright (c) 2017-2018 John Coggeshall                              |
  +----------------------------------------------------------------------+
  | Licensed under the Apache License, Version 2.0 (the "License");      |
  | you may not use this file except in compliance with the License. You |
  | may obtain a copy of the License at:                                 |
  |                                                                      |
  | http://www.apache.org/licenses/LICENSE-2.0                           |
  |                                                                      |
  | Unless required by applicable law or agreed to in writing, software  |
  | distributed under the License is distributed on an "AS IS" BASIS,    |
  | WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or      |
  | implied. See the License for the specific language governing         |
  | permissions and limitations under the License.                       |
  +----------------------------------------------------------------------+
  | Authors: John Coggeshall <john@thissmarthouse.com>                   |
  +----------------------------------------------------------------------+
*/

#include "CoogleEEPROM.h"

void CoogleEEProm::initialize(size_t size)
{

#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("Initializing EEPROM to "));
	Serial.print(size);
	Serial.println(F(" bytes"));
#endif

	EEPROM.begin(size);
}

void CoogleEEProm::initialize()
{
	EEPROM.begin(COOGLE_EEPROM_EEPROM_SIZE);
}

void CoogleEEProm::reset()
{
	return fill(0, COOGLE_EEPROM_EEPROM_SIZE, 0x0);
}

void CoogleEEProm::fill(int startAddress, int endAddress, byte b)
{
	for(int i = startAddress; i <= endAddress; i++) {
		EEPROM.write(i, b);
	}
	
	EEPROM.commit();
}

void CoogleEEProm::dump(int bytesPerRow = 16)
{	
	char buf[10];
	
	if(!Serial) {
		return;
	}
	
	int curRow = 0;
	
	for(int i = 0; i <= COOGLE_EEPROM_EEPROM_SIZE; i++) {
		
		if(curRow == 0) {
			sprintf(buf, "%03X", i);
			Serial.print(buf);
		}
		
		byte b = EEPROM.read(i);
		
		sprintf(buf, "%02X", b);
		
		curRow++;
		
		if(curRow == bytesPerRow) {
			curRow = 0;
			Serial.println(buf);
		} else {
			Serial.print(buf);
		}
	}
}

bool CoogleEEProm::validAddress(int address)
{
#ifdef COOGLEEEPROM_DEBUG
  Serial.printf(F("[COOGLE-EEPROM] %s Address: %d of %d\n"),
                (address <= COOGLE_EEPROM_EEPROM_SIZE) ? "Valid" : "Invalid",
								address, COOGLE_EEPROM_EEPROM_SIZE);
#endif

	return address <= COOGLE_EEPROM_EEPROM_SIZE;
}

bool CoogleEEProm::writeBytes(int startAddress, const byte *array, int length)
{
	if(!validAddress(startAddress) || !validAddress(startAddress + length)) {
		return false;
	}

#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("Writing Bytes: "));
#endif
	
	for(int i = 0; i < length; i++) {

#ifdef COOGLEEEPROM_DEBUG
	Serial.print((char)array[i]);
#endif

		EEPROM.write(startAddress + i, array[i]);
	}

#ifdef COOGLEEEPROM_DEBUG
	Serial.println();
#endif

	EEPROM.commit();

#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("[COOGLE-EEPROM] Wrote "));
	Serial.print(length);
	Serial.print(F(" bytes to address "));
	Serial.println(startAddress);
#endif
	
	return true;
}

bool CoogleEEProm::readBytes(int startAddress, byte array[], int length)
{
	if(!validAddress(startAddress) || !validAddress(startAddress + length)) {
		return false;
	}
	
	for(int i = 0; i < length; i++) {
		array[i] = EEPROM.read(startAddress + i);
	}
	
	return true;
}

bool CoogleEEProm::writeInt(int address, int value)
{
	return writeBytes(address, (byte *)&value, sizeof(value));
}

bool CoogleEEProm::readInt(int address, int *value)
{
	return readBytes(address, (byte *)value, sizeof(int));
}

bool CoogleEEProm::writeString(int address, String str)
{
#ifdef COOGLEEEPROM_DEBUG
        Serial.print(F("[COOGLE-EEPROM] Writing String: "));
        Serial.println(str);
#endif

  return writeString(address, str.c_str());
}

bool CoogleEEProm::writeString(int address, const char *string)
{
	return writeBytes(address, (const byte *)string, strlen(string) + 1);
}

bool CoogleEEProm::writeString(int address, const char *string, int len) {
  return writeBytes(address, (const byte *)string, len);
}

bool CoogleEEProm::readString(int startAddress, char *buffer, int bufSize)
{
#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("Reading into Buffer that is "));
	Serial.print(bufSize);
	Serial.print(F(" byte(s) from "));
	Serial.println(startAddress);

#endif

	if(!validAddress(startAddress)) {

#ifdef COOGLEEPROM_DEBUG
		Serial.println(F("Failed to read from address, invalid address!"));
#endif

		return false;
	}
	
	if(bufSize == 0) {
#ifdef COOGLEEEPROM_DEBUG
		Serial.println(F("Read buffer size was zero, returning false"));
#endif
		return false;
	}
	
	if(bufSize == 1) {

#ifdef COOGLEEEPROM_DEBUG
		Serial.println(F("Buffer Size was 1, returning null"));
#endif

		buffer[0] = '\0';
		return true;
	}
	
	int bufIdx = 0;

#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("[COOGLE-EEPROM] Read Chars: "));
#endif

	do {

		buffer[bufIdx] = EEPROM.read(startAddress + bufIdx);

#ifdef COOGLEEEPROM_DEBUG
		Serial.print(buffer[bufIdx]);
#endif

		bufIdx++;
	
	} while( 
		(buffer[bufIdx - 1] != 0x00) && // Null hit
		(bufIdx < bufSize) && // Out of space
		((startAddress + bufIdx) <= COOGLE_EEPROM_EEPROM_SIZE) // End of EEPROM
	);
	
#ifdef COOGLEEEPROM_DEBUG
	Serial.println();

	if(buffer[bufIdx - 1] == 0x00) {
		Serial.println(F("Read stopped due to NULL"));
	} else if(bufIdx >= bufSize) {
		Serial.println(F("Read stopped due to hitting buffer limit"));
	} else if((startAddress + bufIdx) > COOGLE_EEPROM_EEPROM_SIZE) {
		Serial.println(F("Read stopped due to hitting max EEPROM size limit"));
	}
#endif

	if((buffer[bufIdx - 1] != 0x00) && (bufIdx >= 1)) {
		buffer[bufIdx - 1] = '\0';
	}
	
#ifdef COOGLEEEPROM_DEBUG
	Serial.print(F("Read String: "));
	Serial.println(buffer);
#endif

	return true;
	
}
