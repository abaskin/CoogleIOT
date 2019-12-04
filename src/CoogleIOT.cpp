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

CoogleIOT::CoogleIOT(uint8_t statusPin)
{
    _statusPin = statusPin;
    _serial = false;

    settimeofday_cb([this](){
      _timeSet = true;
      notice(F("Time set to %s"), timeStampISO8601());
    });

    WiFi.onStationModeConnected(
        [=](const WiFiEventStationModeConnected& event) {
          notice(F("Station connected, SSID: %s, BSSID: "
                   "%2X:%2X:%2X:%2X:%2X:%2X, Channel: %d"),
                 event.ssid.c_str(), event.bssid[0], event.bssid[1],
                 event.bssid[2], event.bssid[3], event.bssid[4], event.bssid[5],
                 event.channel);
        });

    WiFi.onStationModeDisconnected(
        [=](const WiFiEventStationModeDisconnected& event) {
          notice(F("Station disconnected, SSID: %s, BSSID: "
                   "%2X:%2X:%2X:%2X:%2X:%2X, Reason: %d"),
                 event.ssid.c_str(), event.bssid[0], event.bssid[1],
                 event.bssid[2], event.bssid[3], event.bssid[4], event.bssid[5],
                 event.reason);
        });

    WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP& event) {
      notice(F("Station IP: %s, Netmask: %s, Gateway: %s"),
             event.ip.toString().c_str(), event.mask.toString().c_str(),
             event.gw.toString().c_str());
      _wifiConnectTime = 0;
      connectToMQTT();
    });
}

CoogleIOT::CoogleIOT()
{
	CoogleIOT(-1);
}

CoogleIOT::~CoogleIOT()
{
	if(logFile) {
		logFile.close();
	}

	SPIFFS.end();
	Serial.end();

	if(_firmwareClientActive) {
		firmwareUpdateTimer.detach();
	}

	heartbeatTimer.detach();
}

// Initialize
bool CoogleIOT::initialize() {
  if (_statusPin > -1) {
    pinMode(_statusPin, OUTPUT);
    flashStatus(COOGLEIOT_STATUS_INIT);
  }

  info(F("Coogle IOT v%s initializing.."), COOGLEIOT_VERSION);

  verifyFlashConfiguration();

  randomSeed(micros());

  EEPROM.begin(COOGLE_EEPROM_SIZE);

  SPIFFS.begin();

  size_t cfgSize;
  EEPROM.get(0, cfgSize);

  DeserializationError err = DeserializationError::InvalidInput;
  if (cfgSize && cfgSize <= cfgDataSize) {
    char buff[cfgDataSize];
    EEPROM.get(sizeof(cfgSize), buff);
    #if COOGLEIOT_USE_MSGPACK
    err = deserializeMsgPack(cfgJson, buff, cfgSize);
    #else
    err = deserializeJson(cfgJson, buff, cfgSize);
    #endif
  }

  if (err) {
    info(F("EEPROM not initialized for platform, initialzing..."));

    deserializeJson(cfgJson, 
      F(R"({                                             )"
        R"( "ap" : { "name" : "", "pass" : "" },         )"
        R"( "remote_ap" : { "name" : "", "pass" : "" },  )"
        R"( "mqtt" : {                                   )"
        R"(   "host" : "",                               )"
        R"(   "user" : "",                               )"
        R"(   "pass" : "",                               )"
        R"(   "client_id" : "",                          )"
        R"(   "port" : 0,                                )"
        R"(   "lwt" : { "topic" : "", "message" : "" }   )"
        R"( },                                           )"
        R"( "fw_upd_url" : ""                            )"
        R"(}                                             )")
    );

    writeConfig();
  }

  if (!(logFile = SPIFFS.open(COOGLEIOT_SPIFFS_LOGFILE, "a+"))) {
    error(F("Could not open SPIFFS log file!"));
  } else {
    info(F("Log file successfully opened"));
  }

  if (strlen(APName()) > 0) {
    WiFi.hostname(APName());
  }

  initializeMQTT();

  connectToSSID();

  timeZone("UTC0");

  enableConfigurationMode();

  if (strlen(FirmwareUpdateUrl()) > 0) {
    firmwareUpdateTimer.attach(COOGLEIOT_FIRMWARE_UPDATE_CHECK, [this]() {
      if (strlen(FirmwareUpdateUrl()) == 0) return;
      if (wifiMulti.run() != WL_CONNECTED) return;

      info(F("Checking for Firmware Updates"));

      LUrlParser::clParseURL URL =
          LUrlParser::clParseURL::ParseURL(FirmwareUpdateUrl());

      if (!URL.IsValid()) return;

      int port;
      if (!URL.GetPort(&port)) {
        port = 80;
      }

      os_intr_lock();

        WiFiClient client;
        auto firmwareUpdateStatus = ESPhttpUpdate.update(client,
                                                         URL.m_Host.c_str(),
                                                         port,
                                                         URL.m_Path.c_str(),
                                                         COOGLEIOT_VERSION);

      os_intr_unlock();

      if (_serial) {
        switch (firmwareUpdateStatus) {
          case HTTP_UPDATE_FAILED:
            warn(F("Warning! Failed to update firmware with specified URL"));
            break;
          case HTTP_UPDATE_NO_UPDATES:
            info(F("Firmware update check completed - at current version"));
            break;
          case HTTP_UPDATE_OK:
            info(F("Firmware Updated!"));
            break;
          default:
            warn(F("Warning! No updated performed. Perhaps an invalid URL?"));
            break;
        }
      }
    });

    info(F("Automatic Firmware Update Enabled"));

    _firmwareClientActive = true;
  }

  heartbeatTimer.attach(COOGLEIOT_HEARTBEAT, [this]() {
    flashStatus(100, 1);
    yield();

    if (_wifiConnectTime &&
        _wifiConnectTime - millis() > COOGLEIOT_MAX_WIFI_CONNECT_TIME &&
        WiFi.status() != WL_CONNECTED) {
      info(
          F("Failed to long to establish a WiFi connection. Restarting "
            "Device."));
      restartDevice();
      return;
    }

    const char* format =
        PSTR(R"({ "timestamp" : "%s", "ip" : "%s", )"
             R"("coogleiot_version" : "%s", "client_id" : "%s" })");
    std::unique_ptr<char[]> json(
        new char[strlen_P(format) + strlen(timeStampISO8601()) +
                 WiFi.localIP().toString().length() +
                 strlen(COOGLEIOT_VERSION) + strlen(MQTTClientId()) + 1]);
    sprintf_P(json.get(), format, timeStampISO8601(),
              WiFi.localIP().toString().c_str(), COOGLEIOT_VERSION,
              MQTTClientId());

    std::unique_ptr<char[]> topic(
        new char[strlen(COOGLEIOT_DEVICE_TOPIC) + strlen(MQTTClientId()) + 2]);
    sprintf_P(topic.get(), PSTR("%s/%s"), COOGLEIOT_DEVICE_TOPIC,
              MQTTClientId());

    MQTTQueueMessage(std::move(topic), std::move(json));
  });

  schedule_function(std::bind(&CoogleIOT::loop, this));

  return true;
}

// Loop
void CoogleIOT::loop() {
  if (wifiMulti.run() != WL_CONNECTED) {
    if (strlen(SSIDName()) > 0) {
      if (!_wifiConnectTime) {
        _wifiConnectTime = millis();
        info(F("Not connected to WiFi. Attempting reconnection."));
        info(F("Will attempt for %s seconds before restarting."),
             COOGLEIOT_MAX_WIFI_CONNECT_TIME);
      }
      return;
    }
  } else {
    connectToMQTT();
    MQTTProcessQueue();
  }

  yield();
  if (webServer) webServer->loop();

  yield();
	#ifndef ARDUINO_ESP8266_ESP01
		if (_dnsServerActive) {
			dnsServer.processNextRequest();
		}
	#endif

	for(auto const& l : loopFunctions) if (!l.second()) loopRemove(l.first);
}

bool CoogleIOT::loopAdd(const char* name, loopFunction_t newFunction) {
  loopFunctions[name] = newFunction;
	return true;
}

bool CoogleIOT::loopRemove(const char* name) {
  auto search = loopFunctions.find(name);
  if (search != loopFunctions.end()) {
    loopFunctions.erase(search);
    return true;
	}
	return false;
}

// Time
char* CoogleIOT::timeStampISO8601() const {
  timeval tv;
  gettimeofday(&tv, nullptr);
  return timeStampISO8601(tv);
}

char* CoogleIOT::timeStampISO8601(timeval& tv) const {
  char buffer[32];  // 2019-11-30T09:57:32.504072+0700
  strftime(buffer, sizeof(buffer), "%FT%T.%%d%z", localtime(&tv.tv_sec));
  char timeStr[32];
  sprintf(timeStr, buffer, tv.tv_usec);
  return std::move(timeStr);
}

char* CoogleIOT::dateISO8601(time_t t) const {
  char buffer[12];  // 2019-11-30
  strftime(buffer, sizeof(buffer), "%F", localtime(&t));
  return std::move(buffer);
}

void CoogleIOT::timeZone(const char* tzName) {
  char tzStr[80];
  for (auto const& tz : tzData) {
    strcpy_P(tzStr, tz);
    if (!strchr(tzStr, ':')) continue;
    auto tzSpec = strchr(tzStr, ':') + 1;
    *(strchr(tzStr, ':')) = '\0';
    if (!strcmp(tzName, tzStr)) {
      _timeSet = false;
      configTime(tzSpec, "pool.ntp.org");
      using esp8266::polledTimeout::oneShotMs;  // import the type to the local
                                                // namespace
      oneShotMs sntpWait(10 * 1000);
      while (!sntpWait) {
        yield();
        if (_timeSet) {
          notice(F("Timezone set to %s"), tzName);
          return;
        }
      }  
      notice(F("Unable to set timezone to %s"), tzName);
      return;
    }
  }
  error(F("Timezone not found: %s"), tzName);
}

float CoogleIOT::tzOffset() const {
  if (!_timeSet) return 0.0;

  timeval tv;
  gettimeofday(&tv, nullptr);
  char buffer[6]; // +0700
  strftime(buffer, sizeof(buffer), "%z", localtime(&tv.tv_sec));
  auto offset = atoi(buffer);
  auto result = offset / 100 + (offset - offset / 100 * 100) / 60.0;
  return (isDST(tv.tv_sec)) ? result - 1.0 : result;
}

bool CoogleIOT::isDST() const {
  timeval tv;
  gettimeofday(&tv, nullptr);
  return isDST(tv.tv_sec);
}

bool CoogleIOT::isDST(time_t currTime) const {
  return localtime(&currTime)->tm_isdst > 0;
}

bool CoogleIOT::timeSet() const {
  return _timeSet;
}

// Reset
void CoogleIOT::restartDevice() {
  _restarting = true;
  ESP.restart();
}

// Logging
CoogleIOT& CoogleIOT::debug(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Debug"), format, arg);
  va_end(arg);
  return *this;
}

CoogleIOT& CoogleIOT::info(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Info"), format, arg);
  va_end(arg);
  return *this;
}

CoogleIOT& CoogleIOT::warn(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Warning"), format, arg);
  va_end(arg);
  return *this;
}

CoogleIOT& CoogleIOT::error(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Error"), format, arg);
  va_end(arg);
  return *this;
}

CoogleIOT& CoogleIOT::critical(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Critical"), format, arg);
  va_end(arg);
  return *this;
}

CoogleIOT& CoogleIOT::notice(const __FlashStringHelper* format, ...) {
  va_list arg;
  va_start(arg, format);
  log(F("Notice"), format, arg);
  va_end(arg);
  return *this;
}

File& CoogleIOT::LogFile()
{
	return logFile;
}

CoogleIOT& CoogleIOT::log(const __FlashStringHelper* severity,
                          const __FlashStringHelper* format, ...) {
  if (!_serial && !logFile) return *this;

  char stackBuffer[msgBufferLen];
  std::unique_ptr<char> buffer;
  buffer.reset(stackBuffer);

  char severityStr[12];
  auto lenHead = snprintf_P(buffer.get(), msgBufferLen, PSTR("[%s %s] "),
                            strcpy_P(severityStr, (PGM_P)severity),
                            timeStampISO8601());

  va_list arg;
  va_start(arg, format);
  auto lenMsg = vsnprintf_P(strchr(buffer.get(), 0), msgBufferLen - lenHead, (PGM_P)format, arg);
  va_end(arg);

  if (lenMsg + lenHead > msgBufferLen - 1) {
    buffer.reset(new char[lenMsg + lenHead + 1]);
    if (!buffer) return *this;
    snprintf_P(buffer.get(), lenMsg + lenHead + 1, PSTR("[%s %s] "),
               severityStr, timeStampISO8601());
    va_start(arg, format);
    vsnprintf(strchr(buffer.get(), 0), lenMsg + 1, (PGM_P)format, arg);
    va_end(arg);
  }

  if (_serial) Serial.println(buffer.get());

  if (logFile) {
    if (logFile.size() + strlen(buffer.get()) > COOGLEIOT_LOGFILE_MAXSIZE) {
      logFile.close();
      SPIFFS.remove(COOGLEIOT_SPIFFS_LOGFILE);
      logFile = SPIFFS.open(COOGLEIOT_SPIFFS_LOGFILE, "a+");

      if (!logFile) {
        error(F("ERROR Could not open SPIFFS log file!"));
        return *this;
      }
    }

    logFile.println(buffer.get());
  }

  return *this;
}

bool CoogleIOT::serialEnabled() const
{
	return _serial;
}

CoogleIOT& CoogleIOT::flashSOS()
{
  using esp8266::polledTimeout::oneShotMs;  // import the type to the local
                                            // namespace
  for (auto i = 3; i; i--) {
    flashStatus(200, 3);
    oneShotMs sosWait(1 * 1000);
    while (!sosWait) yield();

    flashStatus(500, 3);
    sosWait.reset(1 * 1000);
    while (!sosWait) yield();

    flashStatus(200, 3);
    sosWait.reset(5 * 1000);
    while (!sosWait) yield();
        }

	return *this;
}

CoogleIOT& CoogleIOT::resetEEProm()
{
	EEPROM.put(0, (uint32_t)0);

	return *this;
}

bool CoogleIOT::mqttActive() const
{
	return mqttClient.connected();
}

bool CoogleIOT::dnsActive() const
{
  #ifndef ARDUINO_ESP8266_ESP01
		return _dnsServerActive;
	#else
		return false;
	#endif
}

bool CoogleIOT::firmwareClientActive() const
{
	return _firmwareClientActive;
}

bool CoogleIOT::apStatus() const
{
	return _apStatus;
}

const char* CoogleIOT::WiFiStatus() const
{
    switch(WiFi.status()) {
        case WL_CONNECTED:
            return  PSTR("Connected");
            break;
        case WL_NO_SSID_AVAIL:
            return  PSTR("The configured SSID cannot be reached");
            break;
        case WL_CONNECT_FAILED:
            return  PSTR("Failed to connect, Invalid secret");
            break;
        case WL_IDLE_STATUS:
            return  PSTR("WiFi is in process of changing between states");
            break;
        case WL_DISCONNECTED:
        default:
            return  PSTR("Disconnected");
            break;
    }
}

CoogleIOT& CoogleIOT::flashStatus(uint32_t speed) {
	return flashStatus(speed, 5);
}

CoogleIOT& CoogleIOT::flashStatus(uint32_t speed, uint32_t repeat) {
	if (_statusPin > -1) {
		digitalWrite(_statusPin, LOW);

		uint32_t* left = new uint32_t[1];
		*left = repeat * 2 - 1;
		
		schedule_recurrent_function_us([=]() -> bool {
			digitalWrite(_statusPin, !digitalRead(_statusPin));
			if (!--(*left)) {
				delete left;
				return false;
			}
			else return true;
		}, speed * 1000);
	}

	return *this;
}

bool CoogleIOT::verifyFlashConfiguration()
{
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();

	debug(F("Introspecting on-board Flash Memory:"));
	debug(F("Flash ID: %08X"), ESP.getFlashChipId());
	debug(F("Flash real size: %u"), realSize);
	debug(F("Flash IDE Size: %u"), ideSize);
	debug(F("Flash IDE Speed: %u"), ESP.getFlashChipSpeed());
	debug(F("Flash IDE Mode: %u"), (ideMode == FM_QIO ? "QIO" : 
																	ideMode == FM_QOUT ? "QOUT" : 
																	ideMode == FM_DIO ? "DIO" : 
																	ideMode == FM_DOUT ? "DOUT" : 
																	"UNKNOWN"));

	if(ideSize != realSize) {
		warn(F("Flashed size is not equal to size available on chip!"));
		return false;
	}
		
	debug(F("Flash Chip Configuration Verified: OK"));
	return true;
}

void CoogleIOT::enableConfigurationMode()
{
	info(F("Enabling Configuration Mode"));

	initializeLocalAP();

	webServer = std::unique_ptr<CoogleIOTWebserver>(new CoogleIOTWebserver(*this));

	if(!webServer || !webServer->initialize()) {
		error(F("Failed to initialize configuration web server"));
		flashSOS();
	}
}

void CoogleIOT::initializeLocalAP()
{
	IPAddress apLocalIP(192,168,0,1);
	IPAddress apSubnetMask(255,255,255,0);
	IPAddress apGateway(192,168,0,1);

	if(strlen(APPassword()) == 0) {
		info(F("No AP Password found in memory"));
		info(F("Setting to default password: " COOGLEIOT_AP_DEFAULT_PASSWORD));

		APPassword(COOGLEIOT_AP_DEFAULT_PASSWORD);
	}

	if(strlen(APName()) == 0) {
		info(F("No AP Name found in memory. Auto-generating AP name."));

		String localAPName = COOGLEIOT_AP;
		localAPName.concat((int)random(100000, 999999));

		info(F("Setting AP Name To: %s"), localAPName.c_str());

		APName(localAPName.c_str());
	}

	info(F("Intiailzing Access Point"));

	WiFi.softAPConfig(apLocalIP, apGateway, apSubnetMask);
	WiFi.softAP(APName(), APPassword());

	info(F("Local IP Address: %s"), WiFi.softAPIP().toString().c_str());

	#ifndef ARDUINO_ESP8266_ESP01
		if(WiFi.status() != WL_CONNECTED) {

			info(F("Initializing DNS Server"));

			dnsServer.start(COOGLEIOT_DNS_PORT, "*", WiFi.softAPIP());
			_dnsServerActive = true;

		} else {

			info(F("Disabled DNS Server while connected to WiFI"));
			_dnsServerActive = false;

		}
	#endif

	_apStatus = true;

}

// Getters
const char* CoogleIOT::FirmwareUpdateUrl() const {
  return cfgJson[F("fw_upd_url")].as<const char*>();
}

const char* CoogleIOT::MQTTHostname() const {
  return cfgJson[F("mqtt")][F("host")].as<const char*>();
}

const char* CoogleIOT::MQTTClientId() const {
  return cfgJson[F("mqtt")][F("client_id")].as<const char*>();
}

const char* CoogleIOT::MQTTUsername() const {
  return cfgJson[F("mqtt")][F("user")].as<const char*>();
}

const char* CoogleIOT::MQTTPassword() const {
  return cfgJson[F("mqtt")][F("pass")].as<const char*>();
}

const char* CoogleIOT::MQTTLWTTopic() const {
  return cfgJson[F("mqtt")][F("lwt")][F("topic")].as<const char*>();
}

const char* CoogleIOT::MQTTLWTMessage() const {
  return cfgJson[F("mqtt")][F("lwt")][F("message")].as<const char*>();
}

int CoogleIOT::MQTTPort() const {
  return cfgJson[F("mqtt")][F("port")].as<int>();
}

bool CoogleIOT::MQTTRetain() const {
  return mqttRetain;
}

uint8_t CoogleIOT::MQTTQoS() const {
  return mqttQoS;
}

uint8_t CoogleIOT::MQTTQueueSize() const {
  return mqttQueueSize;
}

const char* CoogleIOT::SSIDName() const {
  return cfgJson[F("remote_ap")][F("name")].as<const char*>();
}

const char* CoogleIOT::SSIDPassword() const {
  return cfgJson[F("remote_ap")][F("pass")].as<const char*>();
}

const char* CoogleIOT::APName() const {
  return cfgJson[F("ap")][F("name")].as<const char*>();
}

const char* CoogleIOT::APPassword() const {
  return cfgJson[F("ap")][F("pass")].as<const char*>();
}

// Setters
CoogleIOT& CoogleIOT::MQTTQoS(uint8_t qos) {
  mqttQoS = qos;

  return *this;
}

CoogleIOT& CoogleIOT::MQTTQueueSize(uint8_t qSize) {
  mqttQueueSize = qSize;

  return *this;
}

CoogleIOT& CoogleIOT::FirmwareUpdateUrl(const char* s, bool writeCfg) {
  cfgJson[F("fw_upd_url")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTClientId(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("client_id")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTHostname(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("host")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTPort(int port, bool writeCfg) {
  cfgJson[F("mqtt")][F("port")] = port;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTUsername(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("user")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTPassword(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("pass")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTLWTTopic(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("lwt")][F("topic")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTLWTMessage(const char* s, bool writeCfg) {
  cfgJson[F("mqtt")][F("lwt")][F("message")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::MQTTRetain(bool b) {
  mqttRetain = b;

  return *this;
}

CoogleIOT& CoogleIOT::SSIDName(const char* s, bool writeCfg) {
  cfgJson[F("remote_ap")][F("name")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::SSIDPassword(const char* s, bool writeCfg) {
  cfgJson[F("remote_ap")][F("pass")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::APName(const char* s, bool writeCfg) {
  cfgJson[F("ap")][F("name")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

CoogleIOT& CoogleIOT::APPassword(const char* s, bool writeCfg) {
  cfgJson[F("ap")][F("pass")] = s;
  if (writeCfg) writeConfig();

  return *this;
}

bool CoogleIOT::writeConfig() {
  size_t cfgLength = measureMsgPack(cfgJson);

  if (cfgLength > cfgDataSize) {
    error(F("Config data size (%d) is larger than available EEPROM space (%s)"),
          cfgLength, cfgDataSize);
    return false;
  }

  if (cfgLength == 0) {
    error(F("Config data size is zero, nothing to write to EEPROM"));
    return false;
  }

  char buff[cfgDataSize];
  EEPROM.put(0, cfgLength);
  #if COOGLEIOT_USE_MSGPACK
  serializeMsgPack(cfgJson, buff);
  #else
  serializeJson(cfgJson, buff);
  #endif
  EEPROM.put(sizeof(cfgLength), buff);
  EEPROM.commit();
  return true;
}

// MQTT
CoogleIOT& CoogleIOT::MQTTSubTopic(const char* topic, mqttSubCallback callBack) {
  if (callBack != nullptr)
    mqttSubTopicList.emplace_back(mqttSubTopicEntry {topic, callBack});

  return *this;
}

CoogleIOT& CoogleIOT::MQTTQueueMessage(const char* topicIn, const char* payloadIn) {
	std::unique_ptr<char[]> topic(new char[strlen(topicIn) + 1]);
  std::unique_ptr<char[]> payload(new char[strlen(payloadIn) + 1]);
	strcpy(topic.get(), topicIn);
	strcpy(payload.get(), payloadIn);
  return CoogleIOT::MQTTQueueMessage(std::move(topic), MQTTQoS(), MQTTRetain(),
																		 std::move(payload));
}

CoogleIOT& CoogleIOT::MQTTQueueMessage(std::unique_ptr<char[]> topic,
																			 std::unique_ptr<char[]> payload) {
  return CoogleIOT::MQTTQueueMessage(std::move(topic), MQTTQoS(), MQTTRetain(), 
																		 std::move(payload));
}

CoogleIOT& CoogleIOT::MQTTQueueMessage(std::unique_ptr<char[]> topic, uint8_t qos,
                                       bool retain, std::unique_ptr<char[]> payload) {
	while (mqttPublishQueue.size() >= mqttQueueSize) mqttPublishQueue.pop();
  mqttPublishQueue.emplace(mqttQueueEntry{std::move(topic), std::move(payload),
																				  qos, retain, 0});

  return *this;
}

CoogleIOT& CoogleIOT::MQTTProcessQueue() {
  if (!mqttPublishQueue.empty() && mqttClient.connected()) {
		auto& head = mqttPublishQueue.front();
    if (head.packetId == 0) {
			auto packetId = mqttClient.publish(head.topic.get(), head.qos, head.retain,
																				 head.payload.get());
			if (packetId) head.packetId = packetId;
		}
  }

  return *this;
}

bool CoogleIOT::initializeMQTT()
{
	flashStatus(COOGLEIOT_STATUS_MQTT_INIT);

	if(strlen(MQTTHostname()) == 0) {
		info(F("No MQTT Hostname specified. Cannot Initialize MQTT"));
		return false;
	}

	if(MQTTPort() == 0) {
		info(F("Setting to default MQTT Port: %d"), COOGLEIOT_DEFAULT_MQTT_PORT);
		MQTTPort(COOGLEIOT_DEFAULT_MQTT_PORT);
	}

	if (strlen(MQTTClientId()) == 0) {
		info(F("Setting to default MQTT Client ID: %s"), COOGLEIOT_DEFAULT_MQTT_CLIENT_ID);
		MQTTClientId(COOGLEIOT_DEFAULT_MQTT_CLIENT_ID);
	}

	mqttClient.setClientId(MQTTClientId())
						.setCredentials(MQTTUsername(), MQTTPassword())
						.setServer(MQTTHostname(), MQTTPort());

	if(strlen(MQTTLWTTopic()) != 0)
		mqttClient.setWill(MQTTLWTTopic(), mqttQoS, mqttRetain,
											 MQTTLWTMessage());

	mqttClient.onConnect([this](bool sessionPresent) {
    info(F("Connected to MQTT broker: %s"), MQTTHostname());
    for (const auto& s : mqttSubTopicList) {
			mqttClient.subscribe(s.topic.c_str(), mqttQoS);
			info(F("Subscribed to Topic: %s"), s.topic.c_str());
		}
  });

	mqttClient.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
    info(F("Disconnected from MQTT broker %s, Reason: %d"), MQTTHostname(), reason);
  });

	mqttClient.onPublish([=](uint16_t packetId) {
		if (mqttPublishQueue.front().packetId == packetId) mqttPublishQueue.pop();
	});

	mqttClient.onMessage([=](char* topic, char* payloadIn, 
														AsyncMqttClientMessageProperties properties,
                            size_t len, size_t index, size_t total) {
		static std::string payload;
		static std::vector<bool> pMap;
    static bool multiPart = false;

		if(len != total) {
      if (!multiPart) {
        payload.insert(0, total, ' ');
        std::fill_n(std::back_inserter(pMap), total, false);
        multiPart = true;
      }
			payload.insert(index, payloadIn,len);
			std::fill_n(pMap.begin()+index, len, true);
			if (!std::all_of(pMap.begin(), pMap.end(), [](bool v) { return v; })) return;
      multiPart = false;
		} else
			payload.assign(payloadIn, len);

    auto subMatch = [](const std::string sub, const std::string topic) -> bool {
      auto splitTopic = [](std::string str) -> std::vector<std::string> {
          std::vector<std::string> token;
          size_t pos = 0;
          while ((pos = str.find("/")) != std::string::npos) {
            token.emplace_back(str.substr(0, pos));
            str.erase(0, pos + 1);
          }
          token.emplace_back(str);
          return token;
      };
      if (sub == topic) return true;		
      auto subTok = splitTopic(sub);
      auto topicTok = splitTopic(topic);				
      for (unsigned int t = 0; t < subTok.size(); t++) {
          if (subTok[t] == topicTok[t] || subTok[t] == "+") continue;
          if (subTok[t] == "#" && t == subTok.size() - 1) return true;
          return false;
      }
      return subTok.size() == topicTok.size();
		};

		for (const auto& subT : mqttSubTopicList)
			if (subMatch(subT.topic, topic))
        subT.callBack(topic, payload.c_str(), properties);
	});

  return true;
}

AsyncMqttClient* CoogleIOT::MQTTClient()
{ 
	return &mqttClient;
}

bool CoogleIOT::connectToMQTT()
{
	if(mqttClient.connected()) {
		return true;
	}

	if (_mqttConnecting == true) {
		return false;
	}

	if(WiFi.status() != WL_CONNECTED) {
		info(F("Cannot connect to MQTT because there is no WiFi Connection"));
		return false;
	}

	if(strlen(MQTTHostname()) == 0) {
		return false;
	}

	info(F("Attempting to Connect to MQTT Server"));
  debug(F("MQTT Broker: %s : %d"), MQTTHostname(), MQTTPort());

  schedule_recurrent_function_us([this](){
		if (mqttClient.connected()) {
			_mqttConnecting = false;
			return false;
		}

		mqttClient.connect();
		_mqttConnecting = true;
		return true;
	}, 2 * 1000 * 1000);
	
	return true;
}

bool CoogleIOT::connectToSSID()
{
	flashStatus(COOGLEIOT_STATUS_WIFI_INIT);

	if(strlen(SSIDName()) == 0) {
		info(F("Cannot connect WiFi client, no SSID specified"));
		return false;
	}

	info(F("Connecting to SSID %s"), SSIDName());

	if(strlen(SSIDPassword()) == 0) {
		warn(F("No SSID Password Specified!"));
  	wifiMulti.addAP(SSIDName(), NULL);
	} else {
		wifiMulti.addAP(SSIDName(), SSIDPassword());
	}

	return true;
}

CoogleIOT& CoogleIOT::enableSerial()
{
	return enableSerial(115200);
}

CoogleIOT& CoogleIOT::enableSerial(uint32_t baud) {
  if (!Serial) {
    Serial.begin(baud, SERIAL_8N1, SERIAL_FULL);
    while (!Serial) yield();
  }

  _serial = true;
  return *this;
}
