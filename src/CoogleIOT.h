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

#ifndef COOGLEIOT_H
#define COOGLEIOT_H

#include "EEPROM_map.h"
#include "CoogleIOTConfig.h"
#include <FS.h>

#include "Arduino.h"

#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <vector>

#ifndef ARDUINO_ESP8266_ESP01
#include "DNSServer/DNSServer.h"
#endif

#include "LUrlParser/LUrlParser.h"

#include "CoogleEEPROM.h"
#include "CoogleIOTWebserver.h"

#include <user_interface.h>

    typedef enum {
      DEBUG,
      INFO,
      WARNING,
      ERROR,
      CRITICAL
    } CoogleIOT_LogSeverity;

using sketchtimer_cb_t = std::function<void(void)>;

extern "C" void __coogle_iot_firmware_timer_callback(void*);
extern "C" void __coogle_iot_heartbeat_timer_callback(void *);
extern "C" void __coogle_iot_sketch_timer_callback(void *);

class CoogleIOTWebserver;

class CoogleIOT
{
    public:
     using mqttSubCallback = std::function<void(
        const char*, const char*, AsyncMqttClientMessageProperties)>;

     using mqttSubTopicEntry = struct {
       const std::string topic;
       const mqttSubCallback callBack;
     };

     using mqttQueueEntry = struct {
       std::string topic, payload;
       uint8_t qos;
       bool retain;
       uint16_t packetId;
     };

     bool firmwareUpdateTick = false;
     bool heartbeatTick = false;
     bool sketchTimerTick = false;
     bool _restarting = false;

     CoogleIOT(int);
     CoogleIOT();
     ~CoogleIOT();
     void loop();
     bool initialize();
     CoogleIOT& enableSerial(int, SerialConfig, SerialMode);
     CoogleIOT& enableSerial(int, SerialConfig);
     CoogleIOT& enableSerial(int);
     CoogleIOT& enableSerial();
     AsyncMqttClient* getMQTTClient();

     bool serialEnabled();

     CoogleIOT& flashStatus(int);
     CoogleIOT& flashStatus(int, int);
     CoogleIOT& flashSOS();
     CoogleIOT& resetEEProm();

     void restartDevice();

     const char* RemoteAPName();
     const char* RemoteAPPassword();
     const char* MQTTHostname();
     const char* MQTTUsername();
     const char* MQTTPassword();
     const char* MQTTClientId();
     const char* MQTTLWTTopic();
     const char* MQTTLWTMessage();
     const char* APName();
     const char* APPassword();
     const char* FirmwareUpdateUrl();

     int MQTTPort();
     int MQTTQoS();
     
     String WiFiStatus();
     String TimestampAsString();

     bool verifyFlashConfiguration();
     bool MQTTRetain();
     bool writeConfig();

     CoogleIOT& MQTTPort(int, bool = true);
     CoogleIOT& MQTTHostname(const char*, bool = true);
     CoogleIOT& MQTTUsername(const char* g, bool = true);
     CoogleIOT& MQTTPassword(const char*, bool = true);
     CoogleIOT& MQTTLWTTopic(const char*, bool = true);
     CoogleIOT& MQTTLWTMessage(const char*, bool = true);
     CoogleIOT& MQTTClientId(const char*, bool = true);

     CoogleIOT& MQTTQoS(int);
     CoogleIOT& MQTTRetain(bool);
     CoogleIOT& MQTTSubTopic(const char*, mqttSubCallback);
     CoogleIOT& MQTTQueueMessage(const char*, const char*);
     CoogleIOT& MQTTQueueMessage(const char*, uint8_t, bool, const char*);
     CoogleIOT& MQTTProcessQueue();

     CoogleIOT& RemoteAPName(const char*, bool = true);
     CoogleIOT& RemoteAPPassword(const char*, bool = true);
     CoogleIOT& APName(const char*, bool = true);
     CoogleIOT& APPassword(const char*, bool = true);
     CoogleIOT& FirmwareUpdateUrl(const char*, bool = true);
     CoogleIOT& syncNTPTime(int, int);

     CoogleIOT& warn(String);
     CoogleIOT& error(String);
     CoogleIOT& critical(String);
     CoogleIOT& log(String, CoogleIOT_LogSeverity);
     CoogleIOT& logPrintf(CoogleIOT_LogSeverity, const char* format, ...);
     CoogleIOT& debug(String);
     CoogleIOT& info(String);

     CoogleIOT& registerTimer(int, sketchtimer_cb_t);

     String buildLogMsg(String, CoogleIOT_LogSeverity);
     String Logs(bool);
     String Logs();
     File& LogFile();

     bool mqttActive();
     bool dnsActive();
     bool ntpActive();
     bool firmwareClientActive();
     bool apStatus();

     void checkForFirmwareUpdate();

    private:

        bool _serial;
        int _statusPin;

        HTTPUpdateResult firmwareUpdateStatus;
        time_t now;

#ifndef ARDUINO_ESP8266_ESP01
        DNSServer dnsServer;
#endif

        ESP8266WiFiMulti wifiMulti;
        AsyncMqttClient mqttClient;
        CoogleEEProm eeprom;
        CoogleIOTWebserver *webServer;
        File logFile;

        os_timer_t firmwareUpdateTimer;
        os_timer_t heartbeatTimer;
        os_timer_t sketchTimer;

        int sketchTimerInterval = 0;
        sketchtimer_cb_t sketchTimerCallback;

        int wifiFailuresCount;
        int mqttFailuresCount;

        bool mqttClientActive = false;
        bool dnsServerActive = false;
        bool ntpClientActive = false;
        bool _firmwareClientActive = false;
        bool _apStatus = false;

        void initializeLocalAP();
        void enableConfigurationMode();
        bool connectToSSID();
        bool initializeMQTT();
        bool connectToMQTT();

        bool mqttRetain = COOGLEIOT_DEFAULT_MQTT_RETAIN;
        int mqttQoS = COOGLEIOT_DEFAULT_MQTT_QOS;

        std::vector<mqttSubTopicEntry> mqttSubTopicList;

        std::queue<mqttQueueEntry> mqttPublishQueue;

        DynamicJsonDocument cfgJsonDoc;
        JsonObject cfgJson;

        const char* cfgJsonDefault = R"({
            "ap" : { "name" : "", "pass" : "" },
            "remote_ap" : { "name" : "", "pass" : "" },
            "mqtt" : {
                "host" : "",
                "user" : "",
                "pass" : "",
                "client_id" : "",
                "port" : 0,
                "lwt" : { "topic" : "", "message" : "" }
            },
            "fw_upd_url" : ""
        })";
};

#endif
