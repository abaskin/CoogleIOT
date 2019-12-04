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

#include "CoogleIOTConfig.h"
#include <FS.h>

#include "Arduino.h"

#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <EEPROM.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266httpUpdate.h>
#include <PolledTimeout.h>
#include <Schedule.h>
#include <Ticker.h>
#include <WiFiClient.h>
#include <coredecls.h>   // settimeofday_cb()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>   // struct timeval
#include <time.h>       // time() ctime()

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <vector>
#include <unordered_map>

#ifndef ARDUINO_ESP8266_ESP01
#include "DNSServer/DNSServer.h"
#endif

#include "LUrlParser/LUrlParser.h"

#include "CoogleIOTWebserver.h"

using mqttSubCallback = std::function<void(const char*, const char*,
                                           AsyncMqttClientMessageProperties)>;

using loopFunction_t = std::function<bool(void)>;

constexpr auto cfgDataSize = COOGLE_EEPROM_SIZE - sizeof(size_t);

constexpr auto msgBufferLen = 128;

class CoogleIOTWebserver;

class CoogleIOT
{
  public:
    CoogleIOT(uint8_t);
    CoogleIOT();
    ~CoogleIOT();
    bool initialize();
    void loop();
    bool loopAdd(const char*, loopFunction_t);
    bool loopRemove(const char*);

    char* timeStampISO8601() const;
    char* timeStampISO8601(timeval&) const;
    char* dateISO8601(time_t) const;
    void timeZone(const char*);
    float tzOffset() const;
    bool isDST() const;
    bool isDST(time_t) const;
    bool timeSet() const;

    CoogleIOT& enableSerial(uint32_t);
    CoogleIOT& enableSerial();

    AsyncMqttClient* MQTTClient();

    bool serialEnabled() const;

    CoogleIOT& flashStatus(uint32_t);
    CoogleIOT& flashStatus(uint32_t, uint32_t);
    CoogleIOT& flashSOS();
    CoogleIOT& resetEEProm();

    void restartDevice();

    const char* SSIDName() const;
    const char* SSIDPassword() const;
    const char* MQTTHostname() const;
    const char* MQTTUsername() const;
    const char* MQTTPassword() const;
    const char* MQTTClientId() const;
    const char* MQTTLWTTopic() const;
    const char* MQTTLWTMessage() const;
    const char* APName() const;
    const char* APPassword() const;
    const char* FirmwareUpdateUrl() const;

    int MQTTPort() const;
    uint8_t MQTTQoS() const;
    uint8_t MQTTQueueSize() const;
    bool MQTTRetain() const;

    const char* WiFiStatus() const;

    bool verifyFlashConfiguration();
    bool writeConfig();

    CoogleIOT& MQTTPort(int, bool = true);
    CoogleIOT& MQTTHostname(const char*, bool = true);
    CoogleIOT& MQTTUsername(const char*, bool = true);
    CoogleIOT& MQTTPassword(const char*, bool = true);
    CoogleIOT& MQTTLWTTopic(const char*, bool = true);
    CoogleIOT& MQTTLWTMessage(const char*, bool = true);
    CoogleIOT& MQTTClientId(const char*, bool = true);

    CoogleIOT& MQTTQoS(uint8_t);
    CoogleIOT& MQTTRetain(bool);
    CoogleIOT& MQTTQueueSize(uint8_t);
    CoogleIOT& MQTTSubTopic(const char*, mqttSubCallback);
    CoogleIOT& MQTTQueueMessage(const char*, const char*);
    CoogleIOT& MQTTQueueMessage(std::unique_ptr<char[]>, std::unique_ptr<char[]>);
    CoogleIOT& MQTTQueueMessage(std::unique_ptr<char[]>, uint8_t, bool, std::unique_ptr<char[]>);
    CoogleIOT& MQTTProcessQueue();

    CoogleIOT& SSIDName(const char*, bool = true);
    CoogleIOT& SSIDPassword(const char*, bool = true);
    CoogleIOT& APName(const char*, bool = true);
    CoogleIOT& APPassword(const char*, bool = true);
    CoogleIOT& FirmwareUpdateUrl(const char*, bool = true);

    CoogleIOT& debug(const __FlashStringHelper*, ...);
    CoogleIOT& warn(const __FlashStringHelper*, ...);
    CoogleIOT& error(const __FlashStringHelper*, ...);
    CoogleIOT& critical(const __FlashStringHelper*, ...);
    CoogleIOT& info(const __FlashStringHelper*, ...);
    CoogleIOT& notice(const __FlashStringHelper*, ...);

    File& LogFile();

    bool mqttActive() const;
    bool dnsActive() const;
    bool firmwareClientActive() const;
    bool apStatus() const;

    bool _restarting = false;

  private:
    using mqttSubTopicEntry = struct {
      const std::string topic;
      const mqttSubCallback callBack;
    };

    using mqttQueueEntry = struct {
      std::unique_ptr<char[]> topic, payload;
      uint8_t qos;
      bool retain;
      uint16_t packetId;
    };

    volatile bool flashing = false;

    bool _serial;
    int8_t _statusPin;

#ifndef ARDUINO_ESP8266_ESP01
    DNSServer dnsServer;
#endif

    ESP8266WiFiMulti wifiMulti;
    AsyncMqttClient mqttClient;
    File logFile;

    std::unique_ptr<CoogleIOTWebserver> webServer;

    Ticker firmwareUpdateTimer, heartbeatTimer;

    uint32_t _wifiConnectTime;

    bool _mqttConnecting = false;
    bool _dnsServerActive = false;
    bool _firmwareClientActive = false;
    bool _apStatus = false;
    volatile bool _timeSet = false;

    void initializeLocalAP();
    void enableConfigurationMode();
    bool connectToSSID();
    bool initializeMQTT();
    bool connectToMQTT();

    CoogleIOT& log(const __FlashStringHelper*, const __FlashStringHelper*, ...);

    bool mqttRetain = COOGLEIOT_DEFAULT_MQTT_RETAIN;
    uint8_t mqttQoS = COOGLEIOT_DEFAULT_MQTT_QOS;
    uint8_t mqttQueueSize = COOGLEIOT_DEFAULT_MQTT_QUEUE_SIZE;

    std::vector<mqttSubTopicEntry> mqttSubTopicList;

    std::queue<mqttQueueEntry> mqttPublishQueue;

    std::unordered_map<const char*, loopFunction_t> loopFunctions;

    StaticJsonDocument<COOGLE_CFG_JSON_SIZE> cfgJson;
};

const char tz001[] PROGMEM = "Africa/Abidjan:GMT0";
const char tz002[] PROGMEM = "Africa/Accra:GMT0";
const char tz003[] PROGMEM = "Africa/Addis_Ababa:EAT-3";
const char tz004[] PROGMEM = "Africa/Algiers:CET-1";
const char tz005[] PROGMEM = "Africa/Asmara:EAT-3";
const char tz006[] PROGMEM = "Africa/Asmera:EAT-3";
const char tz007[] PROGMEM = "Africa/Bamako:GMT0";
const char tz008[] PROGMEM = "Africa/Bangui:WAT-1";
const char tz009[] PROGMEM = "Africa/Banjul:GMT0";
const char tz010[] PROGMEM = "Africa/Bissau:GMT0";
const char tz011[] PROGMEM = "Africa/Blantyre:CAT-2";
const char tz012[] PROGMEM = "Africa/Brazzaville:WAT-1";
const char tz013[] PROGMEM = "Africa/Bujumbura:CAT-2";
const char tz014[] PROGMEM = "Africa/Cairo:EEST";
const char tz015[] PROGMEM = "Africa/Casablanca:WET0";
const char tz016[] PROGMEM = "Africa/Ceuta:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz017[] PROGMEM = "Africa/Conakry:GMT0";
const char tz018[] PROGMEM = "Africa/Dakar:GMT0";
const char tz019[] PROGMEM = "Africa/Dar_es_Salaam:EAT-3";
const char tz020[] PROGMEM = "Africa/Djibouti:EAT-3";
const char tz021[] PROGMEM = "Africa/Douala:WAT-1";
const char tz022[] PROGMEM = "Africa/El_Aaiun:WET0";
const char tz023[] PROGMEM = "Africa/Freetown:GMT0";
const char tz024[] PROGMEM = "Africa/Gaborone:CAT-2";
const char tz025[] PROGMEM = "Africa/Harare:CAT-2";
const char tz026[] PROGMEM = "Africa/Johannesburg:SAST-2";
const char tz027[] PROGMEM = "Africa/Kampala:EAT-3";
const char tz028[] PROGMEM = "Africa/Khartoum:EAT-3";
const char tz029[] PROGMEM = "Africa/Kigali:CAT-2";
const char tz030[] PROGMEM = "Africa/Kinshasa:WAT-1";
const char tz031[] PROGMEM = "Africa/Lagos:WAT-1";
const char tz032[] PROGMEM = "Africa/Libreville:WAT-1";
const char tz033[] PROGMEM = "Africa/Lome:GMT0";
const char tz034[] PROGMEM = "Africa/Luanda:WAT-1";
const char tz035[] PROGMEM = "Africa/Lubumbashi:CAT-2";
const char tz036[] PROGMEM = "Africa/Lusaka:CAT-2";
const char tz037[] PROGMEM = "Africa/Malabo:WAT-1";
const char tz038[] PROGMEM = "Africa/Maputo:CAT-2";
const char tz039[] PROGMEM = "Africa/Maseru:SAST-2";
const char tz040[] PROGMEM = "Africa/Mbabane:SAST-2";
const char tz041[] PROGMEM = "Africa/Mogadishu:EAT-3";
const char tz042[] PROGMEM = "Africa/Monrovia:GMT0";
const char tz043[] PROGMEM = "Africa/Nairobi:EAT-3";
const char tz044[] PROGMEM = "Africa/Ndjamena:WAT-1";
const char tz045[] PROGMEM = "Africa/Niamey:WAT-1";
const char tz046[] PROGMEM = "Africa/Nouakchott:GMT0";
const char tz047[] PROGMEM = "Africa/Ouagadougou:GMT0";
const char tz048[] PROGMEM = "Africa/Porto-Novo:WAT-1";
const char tz049[] PROGMEM = "Africa/Sao_Tome:GMT0";
const char tz050[] PROGMEM = "Africa/Timbuktu:GMT0";
const char tz051[] PROGMEM = "Africa/Tripoli:EET-2";
const char tz052[] PROGMEM = "Africa/Tunis:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz053[] PROGMEM = "Africa/Windhoek:WAT-1WAST,M9.1.0,M4.1.0";
const char tz054[] PROGMEM = "America/Adak:HAST10HADT,M3.2.0,M11.1.0";
const char tz055[] PROGMEM = "America/Anchorage:AKST9AKDT,M3.2.0,M11.1.0";
const char tz056[] PROGMEM = "America/Anguilla:AST4";
const char tz057[] PROGMEM = "America/Antigua:AST4";
const char tz058[] PROGMEM = "America/Araguaina:BRT3";
const char tz059[] PROGMEM = "America/Argentina/Buenos_Aires:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz060[] PROGMEM = "America/Argentina/Catamarca:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz061[] PROGMEM = "America/Argentina/ComodRivadavia:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz062[] PROGMEM = "America/Argentina/Cordoba:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz063[] PROGMEM = "America/Argentina/Jujuy:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz064[] PROGMEM = "America/Argentina/La_Rioja:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz065[] PROGMEM = "America/Argentina/Mendoza:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz066[] PROGMEM = "America/Argentina/Rio_Gallegos:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz067[] PROGMEM = "America/Argentina/San_Juan:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz068[] PROGMEM = "America/Argentina/San_Luis:ART3";
const char tz069[] PROGMEM = "America/Argentina/Tucuman:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz070[] PROGMEM = "America/Argentina/Ushuaia:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz071[] PROGMEM = "America/Aruba:AST4";
const char tz072[] PROGMEM = "America/Asuncion:PYT4PYST,M10.3.0/0,M3.2.0/0";
const char tz073[] PROGMEM = "America/Atikokan:EST5";
const char tz074[] PROGMEM = "America/Atka:HAST10HADT,M3.2.0,M11.1.0";
const char tz075[] PROGMEM = "America/Bahia:BRT3";
const char tz076[] PROGMEM = "America/Barbados:AST4";
const char tz077[] PROGMEM = "America/Belem:BRT3";
const char tz078[] PROGMEM = "America/Belize:CST6";
const char tz079[] PROGMEM = "America/Blanc-Sablon:AST4";
const char tz080[] PROGMEM = "America/Boa_Vista:AMT4";
const char tz081[] PROGMEM = "America/Bogota:COT5";
const char tz082[] PROGMEM = "America/Boise:MST7MDT,M3.2.0,M11.1.0";
const char tz083[] PROGMEM = "America/Buenos_Aires:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz084[] PROGMEM = "America/Cambridge_Bay:MST7MDT,M3.2.0,M11.1.0";
const char tz085[] PROGMEM = "America/Campo_Grande:AMT4AMST,M10.2.0/0,M2.3.0/0";
const char tz086[] PROGMEM = "America/Cancun:CST6CDT,M4.1.0,M10.5.0";
const char tz087[] PROGMEM = "America/Caracas:VET4:30";
const char tz088[] PROGMEM = "America/Catamarca:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz089[] PROGMEM = "America/Cayenne:GFT3";
const char tz090[] PROGMEM = "America/Cayman:EST5";
const char tz091[] PROGMEM = "America/Chicago:CST6CDT,M3.2.0,M11.1.0";
const char tz092[] PROGMEM = "America/Chihuahua:MST7MDT,M4.1.0,M10.5.0";
const char tz093[] PROGMEM = "America/Coral_Harbour:EST5";
const char tz094[] PROGMEM = "America/Cordoba:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz095[] PROGMEM = "America/Costa_Rica:CST6";
const char tz096[] PROGMEM = "America/Cuiaba:AMT4AMST,M10.2.0/0,M2.3.0/0";
const char tz097[] PROGMEM = "America/Curacao:AST4";
const char tz098[] PROGMEM = "America/Danmarkshavn:GMT0";
const char tz099[] PROGMEM = "America/Dawson:PST8PDT,M3.2.0,M11.1.0";
const char tz100[] PROGMEM = "America/Dawson_Creek:MST7";
const char tz101[] PROGMEM = "America/Denver:MST7MDT,M3.2.0,M11.1.0";
const char tz102[] PROGMEM = "America/Detroit:EST5EDT,M3.2.0,M11.1.0";
const char tz103[] PROGMEM = "America/Dominica:AST4";
const char tz104[] PROGMEM = "America/Edmonton:MST7MDT,M3.2.0,M11.1.0";
const char tz105[] PROGMEM = "America/Eirunepe:ACT5";
const char tz106[] PROGMEM = "America/El_Salvador:CST6";
const char tz107[] PROGMEM = "America/Ensenada:PST8PDT,M4.1.0,M10.5.0";
const char tz108[] PROGMEM = "America/Fortaleza:BRT3";
const char tz109[] PROGMEM = "America/Fort_Wayne:EST5EDT,M3.2.0,M11.1.0";
const char tz110[] PROGMEM = "America/Glace_Bay:AST4ADT,M3.2.0,M11.1.0";
const char tz111[] PROGMEM = "America/Godthab:WGST";
const char tz112[] PROGMEM = "America/Goose_Bay:AST4ADT,M3.2.0/0:01,M11.1.0/0:01";
const char tz113[] PROGMEM = "America/Grand_Turk:EST5EDT,M3.2.0,M11.1.0";
const char tz114[] PROGMEM = "America/Grenada:AST4";
const char tz115[] PROGMEM = "America/Guadeloupe:AST4";
const char tz116[] PROGMEM = "America/Guatemala:CST6";
const char tz117[] PROGMEM = "America/Guayaquil:ECT5";
const char tz118[] PROGMEM = "America/Guyana:GYT4";
const char tz119[] PROGMEM = "America/Halifax:AST4ADT,M3.2.0,M11.1.0";
const char tz120[] PROGMEM = "America/Havana:CST5CDT,M3.3.0/0,M10.5.0/1";
const char tz121[] PROGMEM = "America/Hermosillo:MST7";
const char tz122[] PROGMEM = "America/Indiana/Indianapolis:EST5EDT,M3.2.0,M11.1.0";
const char tz123[] PROGMEM = "America/Indiana/Knox:CST6CDT,M3.2.0,M11.1.0";
const char tz124[] PROGMEM = "America/Indiana/Marengo:EST5EDT,M3.2.0,M11.1.0";
const char tz125[] PROGMEM = "America/Indiana/Petersburg:EST5EDT,M3.2.0,M11.1.0";
const char tz126[] PROGMEM = "America/Indianapolis:EST5EDT,M3.2.0,M11.1.0";
const char tz127[] PROGMEM = "America/Indiana/Tell_City:CST6CDT,M3.2.0,M11.1.0";
const char tz128[] PROGMEM = "America/Indiana/Vevay:EST5EDT,M3.2.0,M11.1.0";
const char tz129[] PROGMEM = "America/Indiana/Vincennes:EST5EDT,M3.2.0,M11.1.0";
const char tz130[] PROGMEM = "America/Indiana/Winamac:EST5EDT,M3.2.0,M11.1.0";
const char tz131[] PROGMEM = "America/Inuvik:MST7MDT,M3.2.0,M11.1.0";
const char tz132[] PROGMEM = "America/Iqaluit:EST5EDT,M3.2.0,M11.1.0";
const char tz133[] PROGMEM = "America/Jamaica:EST5";
const char tz134[] PROGMEM = "America/Jujuy:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz135[] PROGMEM = "America/Juneau:AKST9AKDT,M3.2.0,M11.1.0";
const char tz136[] PROGMEM = "America/Kentucky/Louisville:EST5EDT,M3.2.0,M11.1.0";
const char tz137[] PROGMEM = "America/Kentucky/Monticello:EST5EDT,M3.2.0,M11.1.0";
const char tz138[] PROGMEM = "America/Knox_IN:CST6CDT,M3.2.0,M11.1.0";
const char tz139[] PROGMEM = "America/La_Paz:BOT4";
const char tz140[] PROGMEM = "America/Lima:PET5";
const char tz141[] PROGMEM = "America/Los_Angeles:PST8PDT,M3.2.0,M11.1.0";
const char tz142[] PROGMEM = "America/Louisville:EST5EDT,M3.2.0,M11.1.0";
const char tz143[] PROGMEM = "America/Maceio:BRT3";
const char tz144[] PROGMEM = "America/Managua:CST6";
const char tz145[] PROGMEM = "America/Manaus:AMT4";
const char tz146[] PROGMEM = "America/Marigot:AST4";
const char tz147[] PROGMEM = "America/Martinique:AST4";
const char tz148[] PROGMEM = "America/Mazatlan:MST7MDT,M4.1.0,M10.5.0";
const char tz149[] PROGMEM = "America/Mendoza:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz150[] PROGMEM = "America/Menominee:CST6CDT,M3.2.0,M11.1.0";
const char tz151[] PROGMEM = "America/Merida:CST6CDT,M4.1.0,M10.5.0";
const char tz152[] PROGMEM = "America/Mexico_City:CST6CDT,M4.1.0,M10.5.0";
const char tz153[] PROGMEM = "America/Miquelon:PMST3PMDT,M3.2.0,M11.1.0";
const char tz154[] PROGMEM = "America/Moncton:AST4ADT,M3.2.0,M11.1.0";
const char tz155[] PROGMEM = "America/Monterrey:CST6CDT,M4.1.0,M10.5.0";
const char tz156[] PROGMEM = "America/Montevideo:UYT3UYST,M10.1.0,M3.2.0";
const char tz157[] PROGMEM = "America/Montreal:EST5EDT,M3.2.0,M11.1.0";
const char tz158[] PROGMEM = "America/Montserrat:AST4";
const char tz159[] PROGMEM = "America/Nassau:EST5EDT,M3.2.0,M11.1.0";
const char tz160[] PROGMEM = "America/New_York:EST5EDT,M3.2.0,M11.1.0";
const char tz161[] PROGMEM = "America/Nipigon:EST5EDT,M3.2.0,M11.1.0";
const char tz162[] PROGMEM = "America/Nome:AKST9AKDT,M3.2.0,M11.1.0";
const char tz163[] PROGMEM = "America/Noronha:FNT2";
const char tz164[] PROGMEM = "America/North_Dakota/Center:CST6CDT,M3.2.0,M11.1.0";
const char tz165[] PROGMEM = "America/North_Dakota/New_Salem:CST6CDT,M3.2.0,M11.1.0";
const char tz166[] PROGMEM = "America/Panama:EST5";
const char tz167[] PROGMEM = "America/Pangnirtung:EST5EDT,M3.2.0,M11.1.0";
const char tz168[] PROGMEM = "America/Paramaribo:SRT3";
const char tz169[] PROGMEM = "America/Phoenix:MST7";
const char tz170[] PROGMEM = "America/Port-au-Prince:EST5";
const char tz171[] PROGMEM = "America/Porto_Acre:ACT5";
const char tz172[] PROGMEM = "America/Port_of_Spain:AST4";
const char tz173[] PROGMEM = "America/Porto_Velho:AMT4";
const char tz174[] PROGMEM = "America/Puerto_Rico:AST4";
const char tz175[] PROGMEM = "America/Rainy_River:CST6CDT,M3.2.0,M11.1.0";
const char tz176[] PROGMEM = "America/Rankin_Inlet:CST6CDT,M3.2.0,M11.1.0";
const char tz177[] PROGMEM = "America/Recife:BRT3";
const char tz178[] PROGMEM = "America/Regina:CST6";
const char tz179[] PROGMEM = "America/Resolute:EST5";
const char tz180[] PROGMEM = "America/Rio_Branco:ACT5";
const char tz181[] PROGMEM = "America/Rosario:ART3ARST,M10.1.0/0,M3.3.0/0";
const char tz182[] PROGMEM = "America/Santiago:CLST";
const char tz183[] PROGMEM = "America/Santo_Domingo:AST4";
const char tz184[] PROGMEM = "America/Sao_Paulo:BRT3BRST,M10.2.0/0,M2.3.0/0";
const char tz185[] PROGMEM = "America/Scoresbysund:EGT1EGST,M3.5.0/0,M10.5.0/1";
const char tz186[] PROGMEM = "America/Shiprock:MST7MDT,M3.2.0,M11.1.0";
const char tz187[] PROGMEM = "America/St_Barthelemy:AST4";
const char tz188[] PROGMEM = "America/St_Johns:NST3:30NDT,M3.2.0/0:01,M11.1.0/0:01";
const char tz189[] PROGMEM = "America/St_Kitts:AST4";
const char tz190[] PROGMEM = "America/St_Lucia:AST4";
const char tz191[] PROGMEM = "America/St_Thomas:AST4";
const char tz192[] PROGMEM = "America/St_Vincent:AST4";
const char tz193[] PROGMEM = "America/Swift_Current:CST6";
const char tz194[] PROGMEM = "America/Tegucigalpa:CST6";
const char tz195[] PROGMEM = "America/Thule:AST4ADT,M3.2.0,M11.1.0";
const char tz196[] PROGMEM = "America/Thunder_Bay:EST5EDT,M3.2.0,M11.1.0";
const char tz197[] PROGMEM = "America/Tijuana:PST8PDT,M4.1.0,M10.5.0";
const char tz198[] PROGMEM = "America/Toronto:EST5EDT,M3.2.0,M11.1.0";
const char tz199[] PROGMEM = "America/Tortola:AST4";
const char tz200[] PROGMEM = "America/Vancouver:PST8PDT,M3.2.0,M11.1.0";
const char tz201[] PROGMEM = "America/Virgin:AST4";
const char tz202[] PROGMEM = "America/Whitehorse:PST8PDT,M3.2.0,M11.1.0";
const char tz203[] PROGMEM = "America/Winnipeg:CST6CDT,M3.2.0,M11.1.0";
const char tz204[] PROGMEM = "America/Yakutat:AKST9AKDT,M3.2.0,M11.1.0";
const char tz205[] PROGMEM = "America/Yellowknife:MST7MDT,M3.2.0,M11.1.0";
const char tz206[] PROGMEM = "Antarctica/Casey:WST-8";
const char tz207[] PROGMEM = "Antarctica/Davis:DAVT-7";
const char tz208[] PROGMEM = "Antarctica/DumontDUrville:DDUT-10";
const char tz209[] PROGMEM = "Antarctica/Mawson:MAWT-6";
const char tz210[] PROGMEM = "Antarctica/McMurdo:NZST-12NZDT,M9.5.0,M4.1.0/3";
const char tz211[] PROGMEM = "Antarctica/Palmer:CLST";
const char tz212[] PROGMEM = "Antarctica/Rothera:ROTT3";
const char tz213[] PROGMEM = "Antarctica/South_Pole:NZST-12NZDT,M9.5.0,M4.1.0/3";
const char tz214[] PROGMEM = "Antarctica/Syowa:SYOT-3";
const char tz215[] PROGMEM = "Antarctica/Vostok:VOST-6";
const char tz216[] PROGMEM = "Arctic/Longyearbyen:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz217[] PROGMEM = "Asia/Aden:AST-3";
const char tz218[] PROGMEM = "Asia/Almaty:ALMT-6";
const char tz219[] PROGMEM = "Asia/Amman:EET-2EEST,M3.5.4/0,M10.5.5/1";
const char tz220[] PROGMEM = "Asia/Anadyr:ANAT-12ANAST,M3.5.0,M10.5.0/3";
const char tz221[] PROGMEM = "Asia/Aqtau:AQTT-5";
const char tz222[] PROGMEM = "Asia/Aqtobe:AQTT-5";
const char tz223[] PROGMEM = "Asia/Ashgabat:TMT-5";
const char tz224[] PROGMEM = "Asia/Ashkhabad:TMT-5";
const char tz225[] PROGMEM = "Asia/Baghdad:AST-3";
const char tz226[] PROGMEM = "Asia/Bahrain:AST-3";
const char tz227[] PROGMEM = "Asia/Baku:AZT-4AZST,M3.5.0/4,M10.5.0/5";
const char tz228[] PROGMEM = "Asia/Bangkok:ICT-7";
const char tz229[] PROGMEM = "Asia/Beirut:EET-2EEST,M3.5.0/0,M10.5.0/0";
const char tz230[] PROGMEM = "Asia/Bishkek:KGT-6";
const char tz231[] PROGMEM = "Asia/Brunei:BNT-8";
const char tz232[] PROGMEM = "Asia/Calcutta:IST-5:30";
const char tz233[] PROGMEM = "Asia/Choibalsan:CHOT-9";
const char tz234[] PROGMEM = "Asia/Chongqing:CST-8";
const char tz235[] PROGMEM = "Asia/Chungking:CST-8";
const char tz236[] PROGMEM = "Asia/Colombo:IST-5:30";
const char tz237[] PROGMEM = "Asia/Dacca:BDT-6";
const char tz238[] PROGMEM = "Asia/Damascus:EET-2EEST,M4.1.5/0,J274/0";
const char tz239[] PROGMEM = "Asia/Dhaka:BDT-6";
const char tz240[] PROGMEM = "Asia/Dili:TLT-9";
const char tz241[] PROGMEM = "Asia/Dubai:GST-4";
const char tz242[] PROGMEM = "Asia/Dushanbe:TJT-5";
const char tz243[] PROGMEM = "Asia/Gaza:EET-2EEST,J91/0,M9.2.4";
const char tz244[] PROGMEM = "Asia/Harbin:CST-8";
const char tz245[] PROGMEM = "Asia/Ho_Chi_Minh:ICT-7";
const char tz246[] PROGMEM = "Asia/Hong_Kong:HKT-8";
const char tz247[] PROGMEM = "Asia/Hovd:HOVT-7";
const char tz248[] PROGMEM = "Asia/Irkutsk:IRKT-8IRKST,M3.5.0,M10.5.0/3";
const char tz249[] PROGMEM = "Asia/Istanbul:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz250[] PROGMEM = "Asia/Jakarta:WIT-7";
const char tz251[] PROGMEM = "Asia/Jayapura:EIT-9";
const char tz252[] PROGMEM = "Asia/Jerusalem:IDDT";
const char tz253[] PROGMEM = "Asia/Kabul:AFT-4:30";
const char tz254[] PROGMEM = "Asia/Kamchatka:PETT-12PETST,M3.5.0,M10.5.0/3";
const char tz255[] PROGMEM = "Asia/Karachi:PKT-5";
const char tz256[] PROGMEM = "Asia/Kashgar:CST-8";
const char tz257[] PROGMEM = "Asia/Katmandu:NPT-5:45";
const char tz258[] PROGMEM = "Asia/Kolkata:IST-5:30";
const char tz259[] PROGMEM = "Asia/Krasnoyarsk:KRAT-7KRAST,M3.5.0,M10.5.0/3";
const char tz260[] PROGMEM = "Asia/Kuala_Lumpur:MYT-8";
const char tz261[] PROGMEM = "Asia/Kuching:MYT-8";
const char tz262[] PROGMEM = "Asia/Kuwait:AST-3";
const char tz263[] PROGMEM = "Asia/Macao:CST-8";
const char tz264[] PROGMEM = "Asia/Macau:CST-8";
const char tz265[] PROGMEM = "Asia/Magadan:MAGT-11MAGST,M3.5.0,M10.5.0/3";
const char tz266[] PROGMEM = "Asia/Makassar:CIT-8";
const char tz267[] PROGMEM = "Asia/Manila:PHT-8";
const char tz268[] PROGMEM = "Asia/Muscat:GST-4";
const char tz269[] PROGMEM = "Asia/Nicosia:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz270[] PROGMEM = "Asia/Novosibirsk:NOVT-6NOVST,M3.5.0,M10.5.0/3";
const char tz271[] PROGMEM = "Asia/Omsk:OMST-6OMSST,M3.5.0,M10.5.0/3";
const char tz272[] PROGMEM = "Asia/Oral:ORAT-5";
const char tz273[] PROGMEM = "Asia/Phnom_Penh:ICT-7";
const char tz274[] PROGMEM = "Asia/Pontianak:WIT-7";
const char tz275[] PROGMEM = "Asia/Pyongyang:KST-9";
const char tz276[] PROGMEM = "Asia/Qatar:AST-3";
const char tz277[] PROGMEM = "Asia/Qyzylorda:QYZT-6";
const char tz278[] PROGMEM = "Asia/Rangoon:MMT-6:30";
const char tz279[] PROGMEM = "Asia/Riyadh:AST-3";
const char tz280[] PROGMEM = "Asia/Riyadh87:zzz-3:07:04";
const char tz281[] PROGMEM = "Asia/Riyadh88:zzz-3:07:04";
const char tz282[] PROGMEM = "Asia/Riyadh89:zzz-3:07:04";
const char tz283[] PROGMEM = "Asia/Saigon:ICT-7";
const char tz284[] PROGMEM = "Asia/Sakhalin:SAKT-10SAKST,M3.5.0,M10.5.0/3";
const char tz285[] PROGMEM = "Asia/Samarkand:UZT-5";
const char tz286[] PROGMEM = "Asia/Seoul:KST-9";
const char tz287[] PROGMEM = "Asia/Shanghai:CST-8";
const char tz288[] PROGMEM = "Asia/Singapore:SGT-8";
const char tz289[] PROGMEM = "Asia/Taipei:CST-8";
const char tz290[] PROGMEM = "Asia/Tashkent:UZT-5";
const char tz291[] PROGMEM = "Asia/Tbilisi:GET-4";
const char tz292[] PROGMEM = "Asia/Tehran:IRDT";
const char tz293[] PROGMEM = "Asia/Tel_Aviv:IDDT";
const char tz294[] PROGMEM = "Asia/Thimbu:BTT-6";
const char tz295[] PROGMEM = "Asia/Thimphu:BTT-6";
const char tz296[] PROGMEM = "Asia/Tokyo:JST-9";
const char tz297[] PROGMEM = "Asia/Ujung_Pandang:CIT-8";
const char tz298[] PROGMEM = "Asia/Ulaanbaatar:ULAT-8";
const char tz299[] PROGMEM = "Asia/Ulan_Bator:ULAT-8";
const char tz300[] PROGMEM = "Asia/Urumqi:CST-8";
const char tz301[] PROGMEM = "Asia/Vientiane:ICT-7";
const char tz302[] PROGMEM = "Asia/Vladivostok:VLAT-10VLAST,M3.5.0,M10.5.0/3";
const char tz303[] PROGMEM = "Asia/Yakutsk:YAKT-9YAKST,M3.5.0,M10.5.0/3";
const char tz304[] PROGMEM = "Asia/Yekaterinburg:YEKT-5YEKST,M3.5.0,M10.5.0/3";
const char tz305[] PROGMEM = "Asia/Yerevan:AMT-4AMST,M3.5.0,M10.5.0/3";
const char tz306[] PROGMEM = "Atlantic/Azores:AZOT1AZOST,M3.5.0/0,M10.5.0/1";
const char tz307[] PROGMEM = "Atlantic/Bermuda:AST4ADT,M3.2.0,M11.1.0";
const char tz308[] PROGMEM = "Atlantic/Canary:WET0WEST,M3.5.0/1,M10.5.0";
const char tz309[] PROGMEM = "Atlantic/Cape_Verde:CVT1";
const char tz310[] PROGMEM = "Atlantic/Faeroe:WET0WEST,M3.5.0/1,M10.5.0";
const char tz311[] PROGMEM = "Atlantic/Faroe:WET0WEST,M3.5.0/1,M10.5.0";
const char tz312[] PROGMEM = "Atlantic/Jan_Mayen:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz313[] PROGMEM = "Atlantic/Madeira:WET0WEST,M3.5.0/1,M10.5.0";
const char tz314[] PROGMEM = "Atlantic/Reykjavik:GMT0";
const char tz315[] PROGMEM = "Atlantic/South_Georgia:GST2";
const char tz316[] PROGMEM = "Atlantic/Stanley:FKT4FKST,M9.1.0,M4.3.0";
const char tz317[] PROGMEM = "Atlantic/St_Helena:GMT0";
const char tz318[] PROGMEM = "Australia/ACT:EST-10EST,M10.1.0,M4.1.0/3";
const char tz319[] PROGMEM = "Australia/Adelaide:CST-9:30CST,M10.1.0,M4.1.0/3";
const char tz320[] PROGMEM = "Australia/Brisbane:EST-10";
const char tz321[] PROGMEM = "Australia/Broken_Hill:CST-9:30CST,M10.1.0,M4.1.0/3";
const char tz322[] PROGMEM = "Australia/Canberra:EST-10EST,M10.1.0,M4.1.0/3";
const char tz323[] PROGMEM = "Australia/Currie:EST-10EST,M10.1.0,M4.1.0/3";
const char tz324[] PROGMEM = "Australia/Darwin:CST-9:30";
const char tz325[] PROGMEM = "Australia/Eucla:CWST-8:45";
const char tz326[] PROGMEM = "Australia/Hobart:EST-10EST,M10.1.0,M4.1.0/3";
const char tz327[] PROGMEM = "Australia/LHI:LHST-10:30LHST-11,M10.1.0,M4.1.0";
const char tz328[] PROGMEM = "Australia/Lindeman:EST-10";
const char tz329[] PROGMEM = "Australia/Lord_Howe:LHST-10:30LHST-11,M10.1.0,M4.1.0";
const char tz330[] PROGMEM = "Australia/Melbourne:EST-10EST,M10.1.0,M4.1.0/3";
const char tz331[] PROGMEM = "Australia/North:CST-9:30";
const char tz332[] PROGMEM = "Australia/NSW:EST-10EST,M10.1.0,M4.1.0/3";
const char tz333[] PROGMEM = "Australia/Perth:WST-8";
const char tz334[] PROGMEM = "Australia/Queensland:EST-10";
const char tz335[] PROGMEM = "Australia/South:CST-9:30CST,M10.1.0,M4.1.0/3";
const char tz336[] PROGMEM = "Australia/Sydney:EST-10EST,M10.1.0,M4.1.0/3";
const char tz337[] PROGMEM = "Australia/Tasmania:EST-10EST,M10.1.0,M4.1.0/3";
const char tz338[] PROGMEM = "Australia/Victoria:EST-10EST,M10.1.0,M4.1.0/3";
const char tz339[] PROGMEM = "Australia/West:WST-8";
const char tz340[] PROGMEM = "Australia/Yancowinna:CST-9:30CST,M10.1.0,M4.1.0/3";
const char tz341[] PROGMEM = "Brazil/Acre:ACT5";
const char tz342[] PROGMEM = "Brazil/DeNoronha:FNT2";
const char tz343[] PROGMEM = "Brazil/East:BRT3BRST,M10.2.0/0,M2.3.0/0";
const char tz344[] PROGMEM = "Brazil/West:AMT4";
const char tz345[] PROGMEM = "Canada/Atlantic:AST4ADT,M3.2.0,M11.1.0";
const char tz346[] PROGMEM = "Canada/Central:CST6CDT,M3.2.0,M11.1.0";
const char tz347[] PROGMEM = "Canada/Eastern:EST5EDT,M3.2.0,M11.1.0";
const char tz348[] PROGMEM = "Canada/East-Saskatchewan:CST6";
const char tz349[] PROGMEM = "Canada/Mountain:MST7MDT,M3.2.0,M11.1.0";
const char tz350[] PROGMEM = "Canada/Newfoundland:NST3:30NDT,M3.2.0/0:01,M11.1.0/0:01";
const char tz351[] PROGMEM = "Canada/Pacific:PST8PDT,M3.2.0,M11.1.0";
const char tz352[] PROGMEM = "Canada/Saskatchewan:CST6";
const char tz353[] PROGMEM = "Canada/Yukon:PST8PDT,M3.2.0,M11.1.0";
const char tz354[] PROGMEM = "Chile/Continental:CLST";
const char tz355[] PROGMEM = "Chile/EasterIsland:EASST";
const char tz356[] PROGMEM = "Etc/GMT:GMT0";
const char tz357[] PROGMEM = "Etc/GMT0:GMT0";
const char tz358[] PROGMEM = "Etc/GMT-0:GMT0";
const char tz359[] PROGMEM = "Etc/GMT+0:GMT0";
const char tz360[] PROGMEM = "Etc/GMT-1:GMT-1";
const char tz361[] PROGMEM = "Etc/GMT+1:GMT+1";
const char tz362[] PROGMEM = "Etc/GMT-10:GMT-10";
const char tz363[] PROGMEM = "Etc/GMT+10:GMT+10";
const char tz364[] PROGMEM = "Etc/GMT-11:GMT-11";
const char tz365[] PROGMEM = "Etc/GMT+11:GMT+11";
const char tz366[] PROGMEM = "Etc/GMT-12:GMT-12";
const char tz367[] PROGMEM = "Etc/GMT+12:GMT+12";
const char tz368[] PROGMEM = "Etc/GMT-13:GMT-13";
const char tz369[] PROGMEM = "Etc/GMT-14:GMT-14";
const char tz370[] PROGMEM = "Etc/GMT-2:GMT-2";
const char tz371[] PROGMEM = "Etc/GMT+2:GMT+2";
const char tz372[] PROGMEM = "Etc/GMT-3:GMT-3";
const char tz373[] PROGMEM = "Etc/GMT+3:GMT+3";
const char tz374[] PROGMEM = "Etc/GMT-4:GMT-4";
const char tz375[] PROGMEM = "Etc/GMT+4:GMT+4";
const char tz376[] PROGMEM = "Etc/GMT-5:GMT-5";
const char tz377[] PROGMEM = "Etc/GMT+5:GMT+5";
const char tz378[] PROGMEM = "Etc/GMT-6:GMT-6";
const char tz379[] PROGMEM = "Etc/GMT+6:GMT+6";
const char tz380[] PROGMEM = "Etc/GMT-7:GMT-7";
const char tz381[] PROGMEM = "Etc/GMT+7:GMT+7";
const char tz382[] PROGMEM = "Etc/GMT-8:GMT-8";
const char tz383[] PROGMEM = "Etc/GMT+8:GMT+8";
const char tz384[] PROGMEM = "Etc/GMT-9:GMT-9";
const char tz385[] PROGMEM = "Etc/GMT+9:GMT+9";
const char tz386[] PROGMEM = "Etc/Greenwich:GMT0";
const char tz387[] PROGMEM = "Etc/UCT:UCT0";
const char tz388[] PROGMEM = "Etc/Universal:UTC0";
const char tz389[] PROGMEM = "Etc/UTC:UTC0";
const char tz390[] PROGMEM = "Etc/Zulu:UTC0";
const char tz391[] PROGMEM = "Europe/Amsterdam:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz392[] PROGMEM = "Europe/Andorra:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz393[] PROGMEM = "Europe/Athens:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz394[] PROGMEM = "Europe/Belfast:GMT0BST,M3.5.0/1,M10.5.0";
const char tz395[] PROGMEM = "Europe/Belgrade:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz396[] PROGMEM = "Europe/Berlin:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz397[] PROGMEM = "Europe/Bratislava:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz398[] PROGMEM = "Europe/Brussels:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz399[] PROGMEM = "Europe/Bucharest:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz400[] PROGMEM = "Europe/Budapest:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz401[] PROGMEM = "Europe/Chisinau:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz402[] PROGMEM = "Europe/Copenhagen:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz403[] PROGMEM = "Europe/Dublin:GMT0IST,M3.5.0/1,M10.5.0";
const char tz404[] PROGMEM = "Europe/Gibraltar:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz405[] PROGMEM = "Europe/Guernsey:GMT0BST,M3.5.0/1,M10.5.0";
const char tz406[] PROGMEM = "Europe/Helsinki:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz407[] PROGMEM = "Europe/Isle_of_Man:GMT0BST,M3.5.0/1,M10.5.0";
const char tz408[] PROGMEM = "Europe/Istanbul:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz409[] PROGMEM = "Europe/Jersey:GMT0BST,M3.5.0/1,M10.5.0";
const char tz410[] PROGMEM = "Europe/Kaliningrad:EET-2EEST,M3.5.0,M10.5.0/3";
const char tz411[] PROGMEM = "Europe/Kiev:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz412[] PROGMEM = "Europe/Lisbon:WET0WEST,M3.5.0/1,M10.5.0";
const char tz413[] PROGMEM = "Europe/Ljubljana:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz414[] PROGMEM = "Europe/London:GMT0BST,M3.5.0/1,M10.5.0";
const char tz415[] PROGMEM = "Europe/Luxembourg:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz416[] PROGMEM = "Europe/Madrid:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz417[] PROGMEM = "Europe/Malta:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz418[] PROGMEM = "Europe/Mariehamn:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz419[] PROGMEM = "Europe/Minsk:EET-2EEST,M3.5.0,M10.5.0/3";
const char tz420[] PROGMEM = "Europe/Monaco:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz421[] PROGMEM = "Europe/Moscow:MSK-3MSD,M3.5.0,M10.5.0/3";
const char tz422[] PROGMEM = "Europe/Nicosia:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz423[] PROGMEM = "Europe/Oslo:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz424[] PROGMEM = "Europe/Paris:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz425[] PROGMEM = "Europe/Podgorica:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz426[] PROGMEM = "Europe/Prague:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz427[] PROGMEM = "Europe/Riga:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz428[] PROGMEM = "Europe/Rome:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz429[] PROGMEM = "Europe/Samara:SAMT-4SAMST,M3.5.0,M10.5.0/3";
const char tz430[] PROGMEM = "Europe/San_Marino:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz431[] PROGMEM = "Europe/Sarajevo:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz432[] PROGMEM = "Europe/Simferopol:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz433[] PROGMEM = "Europe/Skopje:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz434[] PROGMEM = "Europe/Sofia:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz435[] PROGMEM = "Europe/Stockholm:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz436[] PROGMEM = "Europe/Tallinn:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz437[] PROGMEM = "Europe/Tirane:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz438[] PROGMEM = "Europe/Tiraspol:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz439[] PROGMEM = "Europe/Uzhgorod:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz440[] PROGMEM = "Europe/Vaduz:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz441[] PROGMEM = "Europe/Vatican:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz442[] PROGMEM = "Europe/Vienna:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz443[] PROGMEM = "Europe/Vilnius:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz444[] PROGMEM = "Europe/Volgograd:VOLT-3VOLST,M3.5.0,M10.5.0/3";
const char tz445[] PROGMEM = "Europe/Warsaw:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz446[] PROGMEM = "Europe/Zagreb:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz447[] PROGMEM = "Europe/Zaporozhye:EET-2EEST,M3.5.0/3,M10.5.0/4";
const char tz448[] PROGMEM = "Europe/Zurich:CET-1CEST,M3.5.0,M10.5.0/3";
const char tz449[] PROGMEM = "Indian/Antananarivo:EAT-3";
const char tz450[] PROGMEM = "Indian/Chagos:IOT-6";
const char tz451[] PROGMEM = "Indian/Christmas:CXT-7";
const char tz452[] PROGMEM = "Indian/Cocos:CCT-6:30";
const char tz453[] PROGMEM = "Indian/Comoro:EAT-3";
const char tz454[] PROGMEM = "Indian/Kerguelen:TFT-5";
const char tz455[] PROGMEM = "Indian/Mahe:SCT-4";
const char tz456[] PROGMEM = "Indian/Maldives:MVT-5";
const char tz457[] PROGMEM = "Indian/Mauritius:MUT-4";
const char tz458[] PROGMEM = "Indian/Mayotte:EAT-3";
const char tz459[] PROGMEM = "Indian/Reunion:RET-4";
const char tz460[] PROGMEM = "Mexico/BajaNorte:PST8PDT,M4.1.0,M10.5.0";
const char tz461[] PROGMEM = "Mexico/BajaSur:MST7MDT,M4.1.0,M10.5.0";
const char tz462[] PROGMEM = "Mexico/General:CST6CDT,M4.1.0,M10.5.0";
const char tz463[] PROGMEM = "Mideast/Riyadh87:zzz-3:07:04";
const char tz464[] PROGMEM = "Mideast/Riyadh88:zzz-3:07:04";
const char tz465[] PROGMEM = "Mideast/Riyadh89:zzz-3:07:04";
const char tz466[] PROGMEM = "Pacific/Apia:WST11";
const char tz467[] PROGMEM = "Pacific/Auckland:NZST-12NZDT,M9.5.0,M4.1.0/3";
const char tz468[] PROGMEM = "Pacific/Chatham:CHAST-12:45CHADT,M9.5.0/2:45,M4.1.0/3:45";
const char tz469[] PROGMEM = "Pacific/Easter:EASST";
const char tz470[] PROGMEM = "Pacific/Efate:VUT-11";
const char tz471[] PROGMEM = "Pacific/Enderbury:PHOT-13";
const char tz472[] PROGMEM = "Pacific/Fakaofo:TKT10";
const char tz473[] PROGMEM = "Pacific/Fiji:FJT-12";
const char tz474[] PROGMEM = "Pacific/Funafuti:TVT-12";
const char tz475[] PROGMEM = "Pacific/Galapagos:GALT6";
const char tz476[] PROGMEM = "Pacific/Gambier:GAMT9";
const char tz477[] PROGMEM = "Pacific/Guadalcanal:SBT-11";
const char tz478[] PROGMEM = "Pacific/Guam:ChST-10";
const char tz479[] PROGMEM = "Pacific/Honolulu:HST10";
const char tz480[] PROGMEM = "Pacific/Johnston:HST10";
const char tz481[] PROGMEM = "Pacific/Kiritimati:LINT-14";
const char tz482[] PROGMEM = "Pacific/Kosrae:KOST-11";
const char tz483[] PROGMEM = "Pacific/Kwajalein:MHT-12";
const char tz484[] PROGMEM = "Pacific/Majuro:MHT-12";
const char tz485[] PROGMEM = "Pacific/Marquesas:MART9:30";
const char tz486[] PROGMEM = "Pacific/Midway:SST11";
const char tz487[] PROGMEM = "Pacific/Nauru:NRT-12";
const char tz488[] PROGMEM = "Pacific/Niue:NUT11";
const char tz489[] PROGMEM = "Pacific/Norfolk:NFT-11:30";
const char tz490[] PROGMEM = "Pacific/Noumea:NCT-11";
const char tz491[] PROGMEM = "Pacific/Pago_Pago:SST11";
const char tz492[] PROGMEM = "Pacific/Palau:PWT-9";
const char tz493[] PROGMEM = "Pacific/Pitcairn:PST8";
const char tz494[] PROGMEM = "Pacific/Ponape:PONT-11";
const char tz495[] PROGMEM = "Pacific/Port_Moresby:PGT-10";
const char tz496[] PROGMEM = "Pacific/Rarotonga:CKT10";
const char tz497[] PROGMEM = "Pacific/Saipan:ChST-10";
const char tz498[] PROGMEM = "Pacific/Samoa:SST11";
const char tz499[] PROGMEM = "Pacific/Tahiti:TAHT10";
const char tz500[] PROGMEM = "Pacific/Tarawa:GILT-12";
const char tz501[] PROGMEM = "Pacific/Tongatapu:TOT-13";
const char tz502[] PROGMEM = "Pacific/Truk:TRUT-10";
const char tz503[] PROGMEM = "Pacific/Wake:WAKT-12";
const char tz504[] PROGMEM = "Pacific/Wallis:WFT-12";
const char tz505[] PROGMEM = "Pacific/Yap:TRUT-10";
const char tz506[] PROGMEM = "SystemV/AST4:AST4";
const char tz507[] PROGMEM = "SystemV/AST4ADT:AST4ADT,M3.2.0,M11.1.0";
const char tz508[] PROGMEM = "SystemV/CST6:CST6";
const char tz509[] PROGMEM = "SystemV/CST6CDT:CST6CDT,M3.2.0,M11.1.0";
const char tz510[] PROGMEM = "SystemV/EST5:EST5";
const char tz511[] PROGMEM = "SystemV/EST5EDT:EST5EDT,M3.2.0,M11.1.0";
const char tz512[] PROGMEM = "SystemV/HST10:HST10";
const char tz513[] PROGMEM = "SystemV/MST7:MST7";
const char tz514[] PROGMEM = "SystemV/MST7MDT:MST7MDT,M3.2.0,M11.1.0";
const char tz515[] PROGMEM = "SystemV/PST8:PST8";
const char tz516[] PROGMEM = "SystemV/PST8PDT:PST8PDT,M3.2.0,M11.1.0";
const char tz517[] PROGMEM = "SystemV/YST9:GAMT9";
const char tz518[] PROGMEM = "SystemV/YST9YDT:AKST9AKDT,M3.2.0,M11.1.0";
const char tz519[] PROGMEM = "US/Alaska:AKST9AKDT,M3.2.0,M11.1.0";
const char tz520[] PROGMEM = "US/Aleutian:HAST10HADT,M3.2.0,M11.1.0";
const char tz521[] PROGMEM = "US/Arizona:MST7";
const char tz522[] PROGMEM = "US/Central:CST6CDT,M3.2.0,M11.1.0";
const char tz523[] PROGMEM = "US/Eastern:EST5EDT,M3.2.0,M11.1.0";
const char tz524[] PROGMEM = "US/East-Indiana:EST5EDT,M3.2.0,M11.1.0";
const char tz525[] PROGMEM = "US/Hawaii:HST10";
const char tz526[] PROGMEM = "US/Indiana-Starke:CST6CDT,M3.2.0,M11.1.0";
const char tz527[] PROGMEM = "US/Michigan:EST5EDT,M3.2.0,M11.1.0";
const char tz528[] PROGMEM = "US/Mountain:MST7MDT,M3.2.0,M11.1.0";
const char tz529[] PROGMEM = "US/Pacific:PST8PDT,M3.2.0,M11.1.0";
const char tz530[] PROGMEM = "US/Samoa:SST11";

const char* const tzData[] = {
  tz001, tz002, tz003, tz004, tz005, tz006, tz007, tz008, tz009, tz010,
  tz011, tz012, tz013, tz014, tz015, tz016, tz017, tz018, tz019, tz020,
  tz021, tz022, tz023, tz024, tz025, tz026, tz027, tz028, tz029, tz030,
  tz031, tz032, tz033, tz034, tz035, tz036, tz037, tz038, tz039, tz040,
  tz041, tz042, tz043, tz044, tz045, tz046, tz047, tz048, tz049, tz050,
  tz051, tz052, tz053, tz054, tz055, tz056, tz057, tz058, tz059, tz060,
  tz061, tz062, tz063, tz064, tz065, tz066, tz067, tz068, tz069, tz070,
  tz071, tz072, tz073, tz074, tz075, tz076, tz077, tz078, tz079, tz080,
  tz081, tz082, tz083, tz084, tz085, tz086, tz087, tz088, tz089, tz090,
  tz091, tz092, tz093, tz094, tz095, tz096, tz097, tz098, tz099, tz100,
  tz101, tz102, tz103, tz104, tz105, tz106, tz107, tz108, tz109, tz110,
  tz111, tz112, tz113, tz114, tz115, tz116, tz117, tz118, tz119, tz120,
  tz121, tz122, tz123, tz124, tz125, tz126, tz127, tz128, tz129, tz130,
  tz131, tz132, tz133, tz134, tz135, tz136, tz137, tz138, tz139, tz140,
  tz141, tz142, tz143, tz144, tz145, tz146, tz147, tz148, tz149, tz150,
  tz151, tz152, tz153, tz154, tz155, tz156, tz157, tz158, tz159, tz160,
  tz161, tz162, tz163, tz164, tz165, tz166, tz167, tz168, tz169, tz170,
  tz171, tz172, tz173, tz174, tz175, tz176, tz177, tz178, tz179, tz180,
  tz181, tz182, tz183, tz184, tz185, tz186, tz187, tz188, tz189, tz190,
  tz191, tz192, tz193, tz194, tz195, tz196, tz197, tz198, tz199, tz200,
  tz201, tz202, tz203, tz204, tz205, tz206, tz207, tz208, tz209, tz210,
  tz211, tz212, tz213, tz214, tz215, tz216, tz217, tz218, tz219, tz220,
  tz221, tz222, tz223, tz224, tz225, tz226, tz227, tz228, tz229, tz230,
  tz231, tz232, tz233, tz234, tz235, tz236, tz237, tz238, tz239, tz240,
  tz241, tz242, tz243, tz244, tz245, tz246, tz247, tz248, tz249, tz250,
  tz251, tz252, tz253, tz254, tz255, tz256, tz257, tz258, tz259, tz260,
  tz261, tz262, tz263, tz264, tz265, tz266, tz267, tz268, tz269, tz270,
  tz271, tz272, tz273, tz274, tz275, tz276, tz277, tz278, tz279, tz280,
  tz281, tz282, tz283, tz284, tz285, tz286, tz287, tz288, tz289, tz290,
  tz291, tz292, tz293, tz294, tz295, tz296, tz297, tz298, tz299, tz300,
  tz301, tz302, tz303, tz304, tz305, tz306, tz307, tz308, tz309, tz310,
  tz311, tz312, tz313, tz314, tz315, tz316, tz317, tz318, tz319, tz320,
  tz321, tz322, tz323, tz324, tz325, tz326, tz327, tz328, tz329, tz330,
  tz331, tz332, tz333, tz334, tz335, tz336, tz337, tz338, tz339, tz340,
  tz341, tz342, tz343, tz344, tz345, tz346, tz347, tz348, tz349, tz350,
  tz351, tz352, tz353, tz354, tz355, tz356, tz357, tz358, tz359, tz360,
  tz361, tz362, tz363, tz364, tz365, tz366, tz367, tz368, tz369, tz370,
  tz371, tz372, tz373, tz374, tz375, tz376, tz377, tz378, tz379, tz380,
  tz381, tz382, tz383, tz384, tz385, tz386, tz387, tz388, tz389, tz390,
  tz391, tz392, tz393, tz394, tz395, tz396, tz397, tz398, tz399, tz400,
  tz401, tz402, tz403, tz404, tz405, tz406, tz407, tz408, tz409, tz410,
  tz411, tz412, tz413, tz414, tz415, tz416, tz417, tz418, tz419, tz420,
  tz421, tz422, tz423, tz424, tz425, tz426, tz427, tz428, tz429, tz430,
  tz431, tz432, tz433, tz434, tz435, tz436, tz437, tz438, tz439, tz440,
  tz441, tz442, tz443, tz444, tz445, tz446, tz447, tz448, tz449, tz450,
  tz451, tz452, tz453, tz454, tz455, tz456, tz457, tz458, tz459, tz460,
  tz461, tz462, tz463, tz464, tz465, tz466, tz467, tz468, tz469, tz470,
  tz471, tz472, tz473, tz474, tz475, tz476, tz477, tz478, tz479, tz480,
  tz481, tz482, tz483, tz484, tz485, tz486, tz487, tz488, tz489, tz490,
  tz491, tz492, tz493, tz494, tz495, tz496, tz497, tz498, tz499, tz500,
  tz501, tz502, tz503, tz504, tz505, tz506, tz507, tz508, tz509, tz510,
  tz511, tz512, tz513, tz514, tz515, tz516, tz517, tz518, tz519, tz520,
  tz521, tz522, tz523, tz524, tz525, tz526, tz527, tz528, tz529, tz530
};

#endif