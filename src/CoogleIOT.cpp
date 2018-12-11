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

#include "CoogleIOT.h"
#include "CoogleIOTConfig.h"

CoogleIOT* __coogle_iot_self;

extern "C" void __coogle_iot_firmware_timer_callback(void *pArg)
{
	__coogle_iot_self->firmwareUpdateTick = true;
}

extern "C" void __coogle_iot_heartbeat_timer_callback(void *pArg)
{
	__coogle_iot_self->heartbeatTick = true;
}

extern "C" void __coogle_iot_sketch_timer_callback(void *pArg)
{
	__coogle_iot_self->sketchTimerTick = true;
}

CoogleIOT::CoogleIOT(int statusPin)
{
    _statusPin = statusPin;
    _serial = false;
}

CoogleIOT::CoogleIOT()
{
	CoogleIOT(-1);
}

CoogleIOT::~CoogleIOT()
{
	delete webServer;

	if(logFile) {
		logFile.close();
	}

	SPIFFS.end();
	Serial.end();

	if(_firmwareClientActive) {
		os_timer_disarm(&firmwareUpdateTimer);
	}

	if(sketchTimerInterval > 0) {
		os_timer_disarm(&sketchTimer);
	}

	os_timer_disarm(&heartbeatTimer);
}

// Time
CoogleIOT& CoogleIOT::registerTimer(int interval, sketchtimer_cb_t callback)
{
	if(interval <= 0) {
		if(sketchTimerInterval > 0) {
			os_timer_disarm(&sketchTimer);
		}

		sketchTimerCallback = NULL;
		sketchTimerInterval = 0;

		return *this;
	}

	sketchTimerCallback = callback;
	sketchTimerInterval = interval;

	os_timer_setfn(&sketchTimer, __coogle_iot_sketch_timer_callback, NULL);
	os_timer_arm(&sketchTimer, sketchTimerInterval, true);

	return *this;
}

String CoogleIOT::TimestampAsString()
{
	String timestamp;
	struct tm* p_tm;

	if(now) {
		p_tm = localtime(&now);

		timestamp = timestamp +
					(p_tm->tm_year + 1900) + "-" +
					(p_tm->tm_mon < 10 ? "0" : "") + p_tm->tm_mon + "-" +
					(p_tm->tm_mday < 10 ? "0" : "") + p_tm->tm_mday + " " +
					(p_tm->tm_hour < 10 ? "0" : "") + p_tm->tm_hour + ":" +
					(p_tm->tm_min < 10 ? "0" : "") + p_tm->tm_min + ":" +
					(p_tm->tm_sec < 10 ? "0" : "") + p_tm->tm_sec;
	} else {
		timestamp = F("UKWN");
	}

	return timestamp;
}

// Logging
String CoogleIOT::buildLogMsg(String msg, CoogleIOT_LogSeverity severity)
{
	switch(severity) {
		case DEBUG:
			return "[DEBUG " + TimestampAsString() + "] " + msg;
			break;
		case INFO:
			return "[INFO " + TimestampAsString() + "] " + msg;
			break;
		case WARNING:
			return "[WARNING " + TimestampAsString() + "] " + msg;
			break;
		case ERROR:
			return "[ERROR " + TimestampAsString() + "] " + msg;
			break;
		case CRITICAL:
			return "[CRTICAL " + TimestampAsString() + "] " + msg;
			break;
		default:
			return "[UNKNOWN " + TimestampAsString() + "] " + msg;
			break;
	}
}

CoogleIOT& CoogleIOT::logPrintf(CoogleIOT_LogSeverity severity, const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);

    if (len > sizeof(temp) - 1) {
        buffer = new char[len + 1];
        if (!buffer) {
            return *this;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    log(String(buffer), severity);

    if (buffer != temp) {
        delete[] buffer;
    }

    return *this;
}

CoogleIOT& CoogleIOT::debug(String msg)
{
	return log(msg, DEBUG);
}

CoogleIOT& CoogleIOT::info(String msg)
{
	return log(msg, INFO);
}

CoogleIOT& CoogleIOT::warn(String msg)
{
	return log(msg, WARNING);
}

CoogleIOT& CoogleIOT::error(String msg)
{
	return log(msg, ERROR);
}

CoogleIOT& CoogleIOT::critical(String msg)
{
	return log(msg, CRITICAL);
}

File& CoogleIOT::LogFile()
{
	return logFile;
}

String CoogleIOT::Logs()
{
	return CoogleIOT::Logs(false);
}

String CoogleIOT::Logs(bool asHTML)
{
	String retval;

	if(!logFile || !logFile.size()) {
		return retval;
	}

	logFile.seek(0, SeekSet);

	while(logFile.available()) {
		retval += (char)logFile.read();
	}

	logFile.seek(0, SeekEnd);

	return retval;
}

CoogleIOT& CoogleIOT::log(String msg, CoogleIOT_LogSeverity severity)
{
	String logMsg = buildLogMsg(msg, severity);

	if(_serial) {
		Serial.println(logMsg);
	}

	if(!logFile) {
		return *this;
	}

	if((logFile.size() + msg.length()) > COOGLEIOT_LOGFILE_MAXSIZE) {

		logFile.close();
		SPIFFS.remove(COOGLEIOT_SPIFFS_LOGFILE);
		logFile = SPIFFS.open(COOGLEIOT_SPIFFS_LOGFILE, "a+");

		if(!logFile) {
			if(_serial) {
				Serial.println("ERROR Could not open SPIFFS log file!");
			}
			return *this;
		}
	}

	logFile.println(logMsg);

	return *this;
}

bool CoogleIOT::serialEnabled()
{
	return _serial;
}

// Loop
void CoogleIOT::loop()
{
	if(sketchTimerTick) {
		sketchTimerTick = false;
		sketchTimerCallback();
	}

	if(heartbeatTick) {
		heartbeatTick = false;
		flashStatus(100, 1);

		if((wifiFailuresCount > COOGLEIOT_MAX_WIFI_ATTEMPTS) && (WiFi.status() != WL_CONNECTED)) {
			info("Failed too many times to establish a WiFi connection. Restarting Device.");
			restartDevice();
			return;
		}

		if((mqttFailuresCount > COOGLEIOT_MAX_MQTT_ATTEMPTS) && !mqttClient.connected()) {
			info("Failed too many times to establish a MQTT connection. Restarting Device.");
			restartDevice();
			return;
		}

		if(mqttClientActive) {
			char topic[150];
			char json[150];
			
			snprintf(json, 150, "{ \"timestamp\" : \"%s\", \"ip\" : \"%s\", \"coogleiot_version\" : \"%s\", \"client_id\" : \"%s\" }",
					TimestampAsString().c_str(),
					WiFi.localIP().toString().c_str(),
					COOGLEIOT_VERSION,
					MQTTClientId());

			snprintf(topic, 150, COOGLEIOT_DEVICE_TOPIC "/%s", MQTTClientId());

			MQTTQueueMessage(topic, mqttQoS, mqttRetain, json);
		}

	}

	if(wifiMulti.run() != WL_CONNECTED) {
		if(strlen(RemoteAPName()) > 0) {
			info("Not connected to WiFi. Attempting reconnection.");
			if(!connectToSSID()) {
				wifiFailuresCount++;
				logPrintf(INFO, "Attempt %d failed. Will attempt %d times before restarting.", wifiFailuresCount, COOGLEIOT_MAX_WIFI_ATTEMPTS);
				return;
			}
		}

	} else {

		wifiFailuresCount = 0;

		if(!mqttClient.connected()) {
			yield();
			if(!connectToMQTT()) {
				mqttFailuresCount++;
			}
		}

		if(mqttClientActive) {
			mqttFailuresCount = 0;
			MQTTProcessQueue();
			yield();
    }

		if(ntpClientActive) {
			now = time(nullptr);

			if(now) {
				struct tm* p_tm = localtime(&now);

				if( (p_tm->tm_hour == 12) &&
					(p_tm->tm_min == 0) &&
					(p_tm->tm_sec == 6)) {
					yield();
					syncNTPTime(COOGLEIOT_TIMEZONE_OFFSET, COOGLEIOT_DAYLIGHT_OFFSET);
				}
			}
		}

		if(firmwareUpdateTick) {
			firmwareUpdateTick = false;

			checkForFirmwareUpdate();

			if(_serial) {
				switch(firmwareUpdateStatus) {
					case HTTP_UPDATE_FAILED:
						warn("Warning! Failed to update firmware with specified URL");
						break;
					case HTTP_UPDATE_NO_UPDATES:
						info("Firmware update check completed - at current version");
						break;
					case HTTP_UPDATE_OK:
						info("Firmware Updated!");
						break;
					default:
						warn("Warning! No updated performed. Perhaps an invalid URL?");
						break;
				}
			}
		}
	}

	yield();
	webServer->loop();

	yield();
#ifndef ARDUINO_ESP8266_ESP01
	if(dnsServerActive) {
		dnsServer.processNextRequest();
	}
#endif

}

CoogleIOT& CoogleIOT::flashSOS()
{
	for(int i = 0; i < 3; i++) {
		flashStatus(200, 3);
		delay(1000);
		flashStatus(500, 3);
		delay(1000);
		flashStatus(200, 3);
		delay(5000);
	}

	return *this;
}

bool CoogleIOT::mqttActive()
{
	return mqttClientActive;
}

#ifndef ARDUINO_ESP8266_ESP01
bool CoogleIOT::dnsActive()
{
	return dnsServerActive;
}
#else
bool CoogleIOT::dnsActive()
{
	return false;
}
#endif

bool CoogleIOT::ntpActive()
{
	return ntpClientActive;
}

bool CoogleIOT::firmwareClientActive()
{
	return _firmwareClientActive;
}

bool CoogleIOT::apStatus()
{
	return _apStatus;
}

String CoogleIOT::WiFiStatus()
{
    switch(WiFi.status()) {
        case WL_CONNECTED:
            return "Connected";
            break;
        case WL_NO_SSID_AVAIL:
            return "No SSID Available";
            break;
        case WL_CONNECT_FAILED:
            return "Failed to Connect";
            break;
        case WL_IDLE_STATUS:
            return "Idle";
            break;
        case WL_DISCONNECTED:
        default:
            return "Disconnected";
            break;
    }
}

CoogleIOT& CoogleIOT::syncNTPTime(int offsetSeconds, int daylightOffsetSec)
{
	if(!WiFi.status() == WL_CONNECTED) {
		warn("Cannot synchronize time with NTP Servers - No WiFi Connection");
		return *this;
	}

	if(_serial) {
		info("Synchronizing time on device with NTP Servers");
	}

	configTime(offsetSeconds, daylightOffsetSec, COOGLEIOT_NTP_SERVER_1, COOGLEIOT_NTP_SERVER_2, COOGLEIOT_NTP_SERVER_3);

	for(int i = 0; (i < 10) && !time(nullptr); i++) {
		delay(1000);
	}

	if(!(now = time(nullptr))) {
		warn("Failed to synchronize with time server!");
	} else {
		info("Time successfully synchronized with NTP server");
		ntpClientActive = true;
	}

	return *this;
}

CoogleIOT& CoogleIOT::flashStatus(int speed)
{
	flashStatus(speed, 5);
	return *this;
}

CoogleIOT& CoogleIOT::flashStatus(int speed, int repeat)
{
	if(_statusPin > -1) {
		for(int i = 0; i < repeat; i++) {
			digitalWrite(_statusPin, LOW);
			delay(speed);
			digitalWrite(_statusPin, HIGH);
			delay(speed);
		}

		digitalWrite(_statusPin, HIGH);
	}

	return *this;
}

// Initialize
bool CoogleIOT::initialize()
{
	if(_statusPin > -1) {
		pinMode(_statusPin, OUTPUT);
		flashStatus(COOGLEIOT_STATUS_INIT);
	}

	info("Coogle IOT v" COOGLEIOT_VERSION " initializing..");

	verifyFlashConfiguration();

	randomSeed(micros());

	eeprom.initialize(COOGLE_EEPROM_EEPROM_SIZE);

	SPIFFS.begin();

	int cfgSize;
	eeprom.readInt(0, &cfgSize);

	DeserializationError err = DeserializationError::InvalidInput;
	if (cfgSize || cfgSize <= COOGLE_EEPROM_EEPROM_SIZE) {
		std::unique_ptr<char[]> buff(new char[cfgSize]);
		eeprom.readString(sizeof(int), buff.get(), cfgSize);
		err = deserializeMsgPack(cfgJsonDoc, buff.get(), cfgSize);
	}

	if (err) {
		info("EEPROM not initialized for platform, erasing..");

		eeprom.reset();
		SPIFFS.format();

		deserializeJson(cfgJsonDoc, cfgJsonDefault);
		writeConfig();
	}

	cfgJson = cfgJsonDoc.as<JsonObject>();

	logFile = SPIFFS.open(COOGLEIOT_SPIFFS_LOGFILE, "a+");

	if(!logFile) {
		error("Could not open SPIFFS log file!");
	} else {
		info("Log file successfully opened");
	}

	if(strlen(APName()) > 0) {
		WiFi.hostname(APName());
	}

	if(!connectToSSID()) {
		error("Failed to connect to remote AP");
	} else {

		syncNTPTime(COOGLEIOT_TIMEZONE_OFFSET, COOGLEIOT_DAYLIGHT_OFFSET);

		if(!initializeMQTT()) {
			error("Failed to connect to MQTT Server");
		}

	}

	// Used for callbacks into C, where we can't use std::bind to bind an object method
	__coogle_iot_self = this;

	enableConfigurationMode();

	if(strlen(FirmwareUpdateUrl()) > 0) {
		os_timer_setfn(&firmwareUpdateTimer, __coogle_iot_firmware_timer_callback, NULL);
		os_timer_arm(&firmwareUpdateTimer, COOGLEIOT_FIRMWARE_UPDATE_CHECK_MS, true);

		info("Automatic Firmware Update Enabled");

		_firmwareClientActive = true;
	}

	os_timer_setfn(&heartbeatTimer, __coogle_iot_heartbeat_timer_callback, NULL);
	os_timer_arm(&heartbeatTimer, COOGLEIOT_HEARTBEAT_MS, true);

	return true;
}

void CoogleIOT::restartDevice()
{
	_restarting = true;
	ESP.restart();
}

CoogleIOT& CoogleIOT::resetEEProm()
{
	eeprom.reset();
	return *this;
}

bool CoogleIOT::verifyFlashConfiguration()
{
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();

	debug("Introspecting on-board Flash Memory:");
	logPrintf(DEBUG, "Flash ID: %08X", ESP.getFlashChipId());
	logPrintf(DEBUG, "Flash real size: %u", realSize);
	logPrintf(DEBUG, "Flash IDE Size: %u", ideSize);
	logPrintf(DEBUG, "Flash IDE Speed: %u", ESP.getFlashChipSpeed());
	logPrintf(DEBUG, "Flash IDE Mode: %u", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

	if(ideSize != realSize) {
		warn("Flashed size is not equal to size available on chip!");
	} else {
		debug("Flash Chip Configuration Verified: OK");
	}

}

void CoogleIOT::enableConfigurationMode()
{
	info("Enabling Configuration Mode");

	initializeLocalAP();

	webServer = new CoogleIOTWebserver(*this);

	if(!webServer->initialize()) {
		error("Failed to initialize configuration web server");
		flashSOS();
	}
}


void CoogleIOT::initializeLocalAP()
{
	IPAddress apLocalIP(192,168,0,1);
	IPAddress apSubnetMask(255,255,255,0);
	IPAddress apGateway(192,168,0,1);

	if(strlen(APPassword()) == 0) {
		info("No AP Password found in memory");
		info("Setting to default password: " COOGLEIOT_AP_DEFAULT_PASSWORD);

		APPassword(COOGLEIOT_AP_DEFAULT_PASSWORD);
	}

	if(strlen(APName()) == 0) {
		info("No AP Name found in memory. Auto-generating AP name.");

		String localAPName = COOGLEIOT_AP;
		localAPName.concat((int)random(100000, 999999));

		info("Setting AP Name To: " );
		info(localAPName);

		APName(localAPName.c_str());
	}

	info("Intiailzing Access Point");

	WiFi.softAPConfig(apLocalIP, apGateway, apSubnetMask);
	WiFi.softAP(APName(), APPassword());

	info("Local IP Address: ");
	info(WiFi.softAPIP().toString());

#ifndef ARDUINO_ESP8266_ESP01
	if(WiFi.status() != WL_CONNECTED) {

		info("Initializing DNS Server");

		dnsServer.start(COOGLEIOT_DNS_PORT, "*", WiFi.softAPIP());
		dnsServerActive = true;

	} else {

		info("Disabled DNS Server while connected to WiFI");
		dnsServerActive = false;

	}
#endif

	_apStatus = true;

}

// Getters
const char* CoogleIOT::FirmwareUpdateUrl() {
  return cfgJson["fw_upd_url"].as<const char*>();
}

const char* CoogleIOT::MQTTHostname() {
  return cfgJson["mqtt"]["host"].as<const char*>();
}

const char* CoogleIOT::MQTTClientId() {
  return cfgJson["mqtt"]["client_id"].as<const char*>();
}

const char* CoogleIOT::MQTTUsername() {
  return cfgJson["mqtt"]["user"].as<const char*>();
}

const char* CoogleIOT::MQTTPassword() {
  return cfgJson["mqtt"]["pass"].as<const char*>();
}

const char* CoogleIOT::MQTTLWTTopic() {
  return cfgJson["mqtt"]["lwt"]["topic"].as<const char*>();
}

const char* CoogleIOT::MQTTLWTMessage() {
  return cfgJson["mqtt"]["lwt"]["message"].as<const char*>();
}

int CoogleIOT::MQTTPort() {
	return cfgJson["mqtt"]["port"].as<int>();
}

bool CoogleIOT::MQTTRetain() {
  return mqttRetain;
}

int CoogleIOT::MQTTQoS() {
  return mqttQoS;
}

const char* CoogleIOT::RemoteAPName() {
  return cfgJson["remote_ap"]["name"].as<const char*>();
}

const char* CoogleIOT::RemoteAPPassword() {
  return cfgJson["remote_ap"]["pass"].as<const char*>();
}

const char* CoogleIOT::APName() {
  return cfgJson["ap"]["name"].as<const char*>();
}

const char* CoogleIOT::APPassword() {
  return cfgJson["ap"]["pass"].as<const char*>();
}

// Setters
CoogleIOT& CoogleIOT::MQTTQoS(int qos) {
  mqttQoS = qos;

  return *this;
}

CoogleIOT& CoogleIOT::FirmwareUpdateUrl(const char* s, bool writeCfg) {
  cfgJson["fw_upd_url"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTClientId(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["client_id"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTHostname(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["host"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTPort(int port, bool writeCfg) {
  cfgJson["mqtt"]["port"] = port;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTUsername(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["user"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTPassword(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["pass"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTLWTTopic(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["lwt"]["topic"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTLWTMessage(const char* s, bool writeCfg) {
  cfgJson["mqtt"]["lwt"]["message"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTRetain(bool b) {
  mqttRetain = b;

  return *this;
}

CoogleIOT& CoogleIOT::RemoteAPName(const char* s, bool writeCfg) {
  cfgJson["remote_ap"]["name"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::RemoteAPPassword(const char* s, bool writeCfg) {
  cfgJson["remote_ap"]["pass"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::APName(const char* s, bool writeCfg) {
  cfgJson["ap"]["name"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::APPassword(const char* s, bool writeCfg) {
  cfgJson["ap"]["pass"] = s;
  if (writeCfg) writeConfig();

  return *this;
}

bool CoogleIOT::writeConfig() {
  int cfgLength = measureMsgPack(cfgJsonDoc);
  if (cfgLength > COOGLE_EEPROM_EEPROM_SIZE - sizeof(int)) {
    logPrintf(
        ERROR,
        "Config data size (%d) is larger than available EEPROM space (%s)",
        cfgLength, COOGLE_EEPROM_EEPROM_SIZE - sizeof(int));
    return false;
  }

  std::unique_ptr<char[]> buff(new char[cfgLength]);
  eeprom.writeInt(0, cfgLength);
  serializeMsgPack(cfgJsonDoc, buff.get(), cfgLength);
  eeprom.writeString(sizeof(int), buff.get(), cfgLength);
  return true;
}

// MQTT
CoogleIOT& CoogleIOT::MQTTSubTopic(const char* topic,
                                   mqttSubCallback callBack = nullptr) {
  mqttSubTopicList.emplace_back(mqttSubTopicEntry {topic, callBack});

  return *this;
}

CoogleIOT& CoogleIOT::MQTTQueueMessage(const char* topic, const char* payload) {
  return CoogleIOT::MQTTQueueMessage(topic, MQTTQoS(), MQTTRetain(), payload);
}

CoogleIOT& CoogleIOT::MQTTQueueMessage(const char* topic, uint8_t qos,
                                       bool retain, const char* payload) {
  mqttPublishQueue.emplace(mqttQueueEntry{topic, payload, qos, retain, 0});

  return *this;
}

CoogleIOT& CoogleIOT::MQTTProcessQueue() {
  if (!mqttPublishQueue.empty() && mqttClient.connected()) {
		auto& head = mqttPublishQueue.front();
    if (head.packetId == 0) {
			auto packetId = mqttClient.publish(head.topic.c_str(), head.qos, head.retain,
																					head.payload.c_str());
			if (packetId) head.packetId = packetId;
		}
  }

  return *this;
}

void CoogleIOT::checkForFirmwareUpdate()
{
	const char* firmwareUrl = FirmwareUpdateUrl();

	if(strlen(firmwareUrl) == 0) {
		return;
	}

	info("Checking for Firmware Updates");

	os_intr_lock();

	LUrlParser::clParseURL URL = LUrlParser::clParseURL::ParseURL(firmwareUrl);

	if(!URL.IsValid()) {
		os_intr_unlock();
		return;
	}

	int port;
	if(!URL.GetPort(&port)) {
		port = 80;
	}

	firmwareUpdateStatus = ESPhttpUpdate.update(URL.m_Host.c_str(), port, URL.m_Path.c_str(), COOGLEIOT_VERSION);

	os_intr_unlock();
}

bool CoogleIOT::initializeMQTT()
{
	flashStatus(COOGLEIOT_STATUS_MQTT_INIT);

	if(strlen(MQTTHostname()) == 0) {
		info("No MQTT Hostname specified. Cannot Initialize MQTT");
		mqttClientActive = false;
		return false;
	}

	if(MQTTPort() == 0) {
		info("Setting to default MQTT Port");
		MQTTPort(COOGLEIOT_DEFAULT_MQTT_PORT);
	}

	if (strlen(MQTTClientId()) == 0) {
		info(
				"Setting to default MQTT Client "
				"ID: " COOGLEIOT_DEFAULT_MQTT_CLIENT_ID);

		MQTTClientId(COOGLEIOT_DEFAULT_MQTT_CLIENT_ID);
	}

	mqttClient.setClientId(MQTTClientId())
						.setCredentials(MQTTUsername(), MQTTPassword())
						.setServer(MQTTHostname(), MQTTPort());

	if(strlen(MQTTLWTTopic()) != 0)
		mqttClient.setWill(MQTTLWTTopic(), mqttQoS, mqttRetain,
												MQTTLWTMessage());

	mqttClient.onConnect([=](bool sessionPresent) {
    logPrintf(INFO, "Connected to MQTT broker: %s", MQTTHostname());
    for (const auto& s : mqttSubTopicList) {
			mqttClient.subscribe(s.topic.c_str(), mqttQoS);
			logPrintf(INFO, "Subscribed to Topic: %s", s.topic.c_str());
		}
  });

	mqttClient.onPublish([=](uint16_t packetId) {
		if (mqttPublishQueue.front().packetId == packetId) mqttPublishQueue.pop();
	});

	mqttClient.onMessage([=](char* topic, char* payloadIn, 
														AsyncMqttClientMessageProperties properties,
                            size_t len, size_t index, size_t total) {
		auto subMatch = [](const std::string sub, const std::string topic) -> bool {
				auto splitStr = [](std::string str, const char delim) -> std::vector<std::string> {
						std::vector<std::string> v;
						std::string tmp;
						for(auto i = str.begin(); i <= str.end(); ++i)
								if(*i != delim && i != str.end()) tmp += *i;
								else {
										v.emplace_back(tmp);
										tmp = ""; 
								}   
						return v;
				};				
				auto subTok = splitStr(sub,'/');
				auto topicTok = splitStr(topic,'/');				
				for (int t = 0; t < subTok.size(); t++) {
						if (subTok[t] == topicTok[t] || subTok[t] == "+") continue;
						if (subTok[t] == "#" && t == subTok.size() - 1) return true;
						return false;
				}
				return subTok.size() == topicTok.size();;
		};

		static std::string payload;
		static std::vector<bool> pMap;

		if(len != total) {
			payload.insert(0,total,' ');
			std::fill_n(std::back_inserter(pMap), total, false);
			payload.insert(index,payloadIn,len);
			std::fill_n(pMap.begin()+index, len, true);
			if (!std::all_of(pMap.begin(), pMap.end(), [](bool v) { return v; })) return;
		} else
			payload.assign(payloadIn,len);

		for (const auto& subT : mqttSubTopicList)
			if (subT.callBack != nullptr && subMatch(subT.topic,topic))
				subT.callBack(topic,payload.c_str(),properties);

		payload.clear(); pMap.clear();
		payload.shrink_to_fit(); pMap.shrink_to_fit();
	});

  return connectToMQTT();
}

AsyncMqttClient* CoogleIOT::getMQTTClient() { return &mqttClient; }

bool CoogleIOT::connectToMQTT()
{
	if(mqttClient.connected()) {
		mqttClientActive = true;
		return true;
	}

	if(WiFi.status() != WL_CONNECTED) {
		info("Cannot connect to MQTT because there is no WiFi Connection");
		mqttClientActive = false;
		return false;
	}

	if(strlen(MQTTHostname()) == 0) {
		mqttClientActive = false;
		return false;
	}

	info("Attempting to Connect to MQTT Server");

	mqttClient.connect();
  delay(2000);

  logPrintf(DEBUG, "Host: %s : %d",MQTTHostname(), MQTTPort());

	if(!mqttClient.connected()) {
		error("Failed to connect to MQTT Server!");
		mqttClientActive = false;
		return false;
	}

	info("MQTT Client Initialized");
	mqttClientActive = true;
	return true;
}

bool CoogleIOT::connectToSSID()
{
	flashStatus(COOGLEIOT_STATUS_WIFI_INIT);

	if(strlen(RemoteAPName()) == 0) {
		info("Cannot connect WiFi client, no remote AP specified");
		return false;
	}

	info("Connecting to remote AP");

	if(strlen(RemoteAPPassword()) == 0) {
		warn("No Remote AP Password Specified!");
  	wifiMulti.addAP(RemoteAPName(), NULL);
	} else {
		wifiMulti.addAP(RemoteAPName(), RemoteAPPassword());
	}

  int waitLoop = 50;
  while (int wifiStatus = wifiMulti.run() != WL_CONNECTED && waitLoop) {
    if (wifiStatus == WL_CONNECT_FAILED) {
      error("Invalid secret");
			break;
    }
    switch (wifiStatus) {
      case WL_IDLE_STATUS:    info("Status: Wi-Fi is in process of changing between statuses");
                              break;
      case WL_NO_SSID_AVAIL:  info("Status: configured SSID cannot be reached");
                              break;
      case WL_CONNECTED:      info("Status: successful connection is established");
                              break;
      case WL_DISCONNECTED:   info("Status: module is not configured in station mode");
                              break;
      default: break;
    }
    delay(500);
    waitLoop--;
  }

	if(WiFi.status() != WL_CONNECTED) {
		error("Could not connect to Access Point!");
		flashSOS();

		return false;
	}

	info("Connected to Remote Access Point!");
	info("Our IP Address is:");
	info(WiFi.localIP().toString());

	return true;
}

CoogleIOT& CoogleIOT::enableSerial()
{
	return enableSerial(115200);
}

CoogleIOT& CoogleIOT::enableSerial(int baud)
{
    if(!Serial) {

      Serial.begin(baud);

      while(!Serial) {
          yield();
      }

    }

    _serial = true;
    return *this;
}

CoogleIOT& CoogleIOT::enableSerial(int baud, SerialConfig config)
{
    if(!Serial) {

      Serial.begin(baud, config);

      while(!Serial) {
          yield();
      }

    }

    _serial = true;
    return *this;
}

CoogleIOT& CoogleIOT::enableSerial(int baud, SerialConfig config, SerialMode mode)
{
    if(!Serial) {

      Serial.begin(baud, config, mode);

      while(!Serial) {
          yield();
      }

    }

    _serial = true;
    return *this;
}
