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

#ifndef COOGLEIOT_EEPROM_MAP
#define COOGLEIOT_EEPROM_MAP

#define COOGLEIOT_AP_PASSWORD_MAXLEN 64
#define COOGLEIOT_AP_NAME_MAXLEN 32

#define COOGLEIOT_REMOTE_AP_PASSWORD_MAXLEN 64 
#define COOGLEIOT_REMOTE_AP_NAME_MAXLEN 32    

#define COOGLEIOT_MQTT_HOST_MAXLEN 64
#define COOGLEIOT_MQTT_USER_MAXLEN 32 
#define COOGLEIOT_MQTT_USER_PASSWORD_MAXLEN 64
#define COOGLEIOT_MQTT_CLIENT_ID_MAXLEN 32 
#define COOGLEIOT_MQTT_PORT_MAXLEN 6 
#define COOGLEIOT_MQTT_LWT_TOPIC_MAXLEN 128 
#define COOGLEIOT_MQTT_LWT_MESSAGE_MAXLEN 256

#define COOGLEIOT_FIRMWARE_UPDATE_URL_MAXLEN 255

#endif
