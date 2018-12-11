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

#include <CoogleIOTWebserver.h>
#include <ESP8266WebServer.h>

#ifdef COOGLEIOT_DEBUG

#define MDNS_DEBUG_RX
#define MDNS_DEBUG_TX
#define MDNS_DEBUG_ERR

#endif

CoogleIOTWebserver::CoogleIOTWebserver(CoogleIOT &_iot)
{
	setIOT(_iot);

	this->serverPort = 80;

	iot->info("Creating Configuration Web Server");

	setWebserver(new ESP8266WebServer(this->serverPort));
}

CoogleIOTWebserver::CoogleIOTWebserver(CoogleIOT& _iot, int port)
{
	setIOT(_iot);

	this->serverPort = port;

	iot->info("Creating Configuration Web Server");
	setWebserver(new ESP8266WebServer(this->serverPort));
}

CoogleIOTWebserver::~CoogleIOTWebserver()
{
	delete webServer;
}

CoogleIOTWebserver& CoogleIOTWebserver::initializePages()
{
	webServer->on("/", std::bind(&CoogleIOTWebserver::handleRoot, this));
	webServer->on("/css", std::bind(&CoogleIOTWebserver::handleCSS, this));
	webServer->on("/reset", std::bind(&CoogleIOTWebserver::handleReset, this));
	webServer->on("/restart", std::bind(&CoogleIOTWebserver::handleRestart, this));
	webServer->on("/jquery", std::bind(&CoogleIOTWebserver::handleJS, this));
	webServer->on("/logs", std::bind(&CoogleIOTWebserver::handleLogs, this));

	webServer->on("/api/status", std::bind(&CoogleIOTWebserver::handleApiStatus, this));
	webServer->on("/api/reset", std::bind(&CoogleIOTWebserver::handleApiReset, this));
	webServer->on("/api/restart", std::bind(&CoogleIOTWebserver::handleApiRestart, this));
	webServer->on("/api/save", std::bind(&CoogleIOTWebserver::handleSubmit, this));

	webServer->on("/firmware-upload",
				  HTTP_POST,
				  std::bind(&CoogleIOTWebserver::handleFirmwareUploadResponse, this),
				  std::bind(&CoogleIOTWebserver::handleFirmwareUpload, this)
	);

	webServer->onNotFound(std::bind(&CoogleIOTWebserver::handle404, this));

	return *this;
}

CoogleIOTWebserver& CoogleIOTWebserver::setServerPort(int port)
{
	this->serverPort = port;
	return *this;
}

CoogleIOTWebserver& CoogleIOTWebserver::setWebserver(ESP8266WebServer* server)
{
	this->webServer = server;
	return *this;
}

CoogleIOTWebserver& CoogleIOTWebserver::setIOT(CoogleIOT& _iot)
{
	this->iot = &_iot;
	return *this;
}

bool CoogleIOTWebserver::initialize()
{
	iot->info("Initializing Webserver");

	initializePages();
	webServer->begin();

	iot->info("Webserver Initiailized!");

	return true;
}

void CoogleIOTWebserver::loop()
{
	webServer->handleClient();
}

String CoogleIOTWebserver::htmlEncode(const char *input)
{
	return htmlEncode(String(input));
}

String CoogleIOTWebserver::htmlEncode(String input)
{
	char t;
	String retval, escape;

	for(int i = 0; i < input.length(); i++) {
		t = input.charAt(i);
		switch(t) {
			case '&':
				escape = "&amp;";
				break;

			case '<':
				escape = "&lt;";
				break;

			case '>':
				escape = "&gt;";
				break;

			case '"':
				escape = "&quot;";
				break;

			case '\'':
				escape = "&#39;";
				break;

			default:
				escape = t;
				break;
		}

		retval = retval + escape;
	}

	return retval;
}
void CoogleIOTWebserver::handleRoot()
{
	String page(FPSTR(WEBPAGE_Home));

	page.replace(F("{{ap_name}}"), htmlEncode(iot->APName()));
	page.replace(F("{{ap_password}}"), htmlEncode(iot->APPassword()));
	page.replace(F("{{remote_ap_name}}"), htmlEncode(iot->RemoteAPName()));
	page.replace(F("{{remote_ap_password}}"), htmlEncode(iot->RemoteAPPassword()));
	page.replace(F("{{mqtt_host}}"), htmlEncode(iot->MQTTHostname()));
	page.replace(F("{{mqtt_username}}"), htmlEncode(iot->MQTTUsername()));
	page.replace(F("{{mqtt_password}}"), htmlEncode(iot->MQTTPassword()));
	page.replace(F("{{mqtt_client_id}}"), htmlEncode(iot->MQTTClientId()));
	page.replace(F("{{mqtt_lwt_topic}}"), htmlEncode(iot->MQTTLWTTopic()));
	page.replace(F("{{mqtt_lwt_message}}"), htmlEncode(iot->MQTTLWTMessage()));
	page.replace(F("{{firmware_url}}"), htmlEncode(iot->FirmwareUpdateUrl()));
	page.replace(F("{{mqtt_port}}"), htmlEncode(String(iot->MQTTPort())));
	page.replace(F("{{coogleiot_version}}"), htmlEncode(COOGLEIOT_VERSION));
	page.replace(F("{{coogleiot_buildtime}}"), htmlEncode(__DATE__ " " __TIME__));
	page.replace(F("{{coogleiot_ap_ssid}}"), htmlEncode(iot->APName()));
	page.replace(F("{{wifi_ip_address}}"), htmlEncode(WiFi.localIP().toString()));
	page.replace(F("{{mac_address}}"), htmlEncode(WiFi.macAddress()));
	page.replace(F("{{wifi_status}}"), htmlEncode(iot->WiFiStatus()));
	page.replace(F("{{mqtt_status}}"), iot->mqttActive() ? "Active" : "Not Connected");
	page.replace(F("{{ntp_status}}"), iot->ntpActive() ? "Active" : "Not Connected");
	page.replace(F("{{dns_status}}"), iot->dnsActive() ? "Active" : "Disabled");
	page.replace(F("{{firmware_update_status}}"), iot->firmwareClientActive() ? "Active" : "Disabled");
	page.replace(F("{{coogleiot_ap_status}}"), iot->apStatus() ? "Active" : "Disabled");

  webServer->send(200, "text/html", page.c_str());
}

void CoogleIOTWebserver::handleJS()
{
	webServer->sendHeader("Content-Encoding", "gzip", true);
	webServer->send_P(200, "application/javascript", jquery_3_2_1_min_js_gz, jquery_3_2_1_min_js_gz_len);
}

void CoogleIOTWebserver::handleCSS()
{
	webServer->send_P(200, "text/css", WEBPAGE_CSS, mini_default_min_css_len);
}

void CoogleIOTWebserver::handle404()
{
	webServer->send_P(404, "text/html", WEBPAGE_NOTFOUND);
}

void CoogleIOTWebserver::handleLogs()
{
	File logFile;

	logFile = iot->LogFile();

	logFile.seek(0, SeekSet);
	webServer->streamFile(logFile, "text/html");
	logFile.seek(0, SeekEnd);
}

void CoogleIOTWebserver::handleFirmwareUploadResponse()
{
	if(_manualFirmwareUpdateSuccess) {
		webServer->send_P(200, "text/html", WEBPAGE_Restart);
		return;
	}

	webServer->send(200, "text/html", "There was an error uploading the firmware");

}

void CoogleIOTWebserver::handleFirmwareUpload()
{
	HTTPUpload& upload = webServer->upload();
	uint32_t maxSketchSpace;

	switch(upload.status) {
		case UPLOAD_FILE_START:
			WiFiUDP::stopAll();

			iot->info("Receiving Firmware Upload...");

			maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

			if(!Update.begin(maxSketchSpace)) {
				iot->error("Failed to begin Firmware Upload!");

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}

			break;
		case UPLOAD_FILE_WRITE:

			if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {

				iot->error("Failed to write Firmware Upload!");

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}
			break;

		case UPLOAD_FILE_END:

			if(Update.end(true)) {

				iot->info("Firmware updated!");

				_manualFirmwareUpdateSuccess = true;

			} else {
				iot->error("Failed to update Firmware!");

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}

			break;

		case UPLOAD_FILE_ABORTED:
			Update.end();

			iot->info("Firmware upload aborted!");

			_manualFirmwareUpdateSuccess = false;

			break;
	}

	yield();
}

void CoogleIOTWebserver::handleSubmit()
{
  StaticJsonDocument<200> jsonBuffer;
  WiFiClientPrint<> p(webServer->client());

  JsonObject retval = jsonBuffer.to<JsonObject>();
  JsonArray errors = retval.createNestedArray("errors");

  bool success = true;

  if (webServer->arg("ap_name").length() > 0) {
    if (webServer->arg("ap_name").length() < COOGLEIOT_AP_NAME_MAXLEN) {
      iot->APName(webServer->arg("ap_name").c_str(),false);
    } else {
      errors.add("AP Name was too long");
      success = false;
    }
	}

	if (webServer->arg("ap_password").length() < COOGLEIOT_AP_PASSWORD_MAXLEN) {
          iot->APPassword(webServer->arg("ap_password").c_str(), false);
        } else {
		errors.add("AP Password is too long!");
		success = false;
	}

	if (webServer->arg("remote_ap_name").length() > 0) {
		if (webServer->arg("remote_ap_name").length() < COOGLEIOT_REMOTE_AP_NAME_MAXLEN) {
			iot->RemoteAPName(webServer->arg("remote_ap_name").c_str(),false);
		} else {
			errors.add("Remote AP Name is too long!");
			success = false;
		}
	}

	if (webServer->arg("remote_ap_password").length() < COOGLEIOT_REMOTE_AP_PASSWORD_MAXLEN) {
		iot->RemoteAPPassword(webServer->arg("remote_ap_password").c_str(),false);
	} else {
		errors.add("Remote AP Password was too long!");
		success = false;
	}

	if (webServer->arg("mqtt_host").length() > 0) {
		if (webServer->arg("mqtt_host").length() < COOGLEIOT_MQTT_HOST_MAXLEN) {
			iot->MQTTHostname(webServer->arg("mqtt_host").c_str(),false);
		} else {
			errors.add("The MQTT Hostname was too long!");
			success = false;
		}
	}

	if (webServer->arg("mqtt_port").length() > 0) {
		if (webServer->arg("mqtt_port").toInt() > 0) {
			iot->MQTTPort(webServer->arg("mqtt_port").toInt(),false);
		} else {
			errors.add("The MQTT Port was Invalid");
			success = false;
		}
	}

	if (webServer->arg("mqtt_username").length() > 0) {
		if (webServer->arg("mqtt_username").length() < COOGLEIOT_MQTT_USER_MAXLEN) {
			iot->MQTTUsername(webServer->arg("mqtt_username").c_str(),false);
		} else {
			errors.add("The MQTT Username was too long");
			success = false;
		}
	}

	if (webServer->arg("mqtt_password").length() > 0) {
		if (webServer->arg("mqtt_password").length() < COOGLEIOT_MQTT_USER_PASSWORD_MAXLEN) {
			iot->MQTTPassword(webServer->arg("mqtt_password").c_str(),false);
		} else {
			errors.add("The MQTT Password was too long");
			success = false;
		}
	}

	if (webServer->arg("mqtt_client_id").length() > 0) {
		if (webServer->arg("mqtt_client_id").length() < COOGLEIOT_MQTT_CLIENT_ID_MAXLEN) {
			iot->MQTTClientId(webServer->arg("mqtt_client_id").c_str(),false);
		} else {
			errors.add("The MQTT Client ID was too long");
			success = false;
		}
	}

	if (webServer->arg("mqtt_lwt_topic").length() > 0) {
		if (webServer->arg("mqtt_lwt_topic").length() < COOGLEIOT_MQTT_LWT_TOPIC_MAXLEN) {
			iot->MQTTLWTTopic(webServer->arg("mqtt_lwt_topic").c_str(),false);
		} else {
			errors.add("The MQTT LWT topic was too long");
			success = false;
		}
	}

	if (webServer->arg("mqtt_lwt_message").length() > 0) {
		if (webServer->arg("mqtt_lwt_message").length() < COOGLEIOT_MQTT_LWT_MESSAGE_MAXLEN) {
			iot->MQTTLWTMessage(webServer->arg("mqtt_lwt_message").c_str(),false);
		} else {
			errors.add("The MQTT LWT message was too long");
			success = false;
		}
	}

	if (webServer->arg("firmware_url").length() > 0) {
		if (webServer->arg("firmware_url").length() < COOGLEIOT_FIRMWARE_UPDATE_URL_MAXLEN) {
			iot->FirmwareUpdateUrl(webServer->arg("firmware_url").c_str(),false);
		} else {
			errors.add("The Firmware Update URL was too long");
			success = false;
		}
	}

	iot->writeConfig();

	retval["status"] = success;

	webServer->setContentLength(retval.size());
	webServer->send(200, "application/json", "");

	String pStr;
  serializeJson(retval,pStr);
	p.print(pStr);
	p.stop();
}

void CoogleIOTWebserver::handleReset()
{
	webServer->send_P(200, "text/html", WEBPAGE_Restart);
	iot->resetEEProm();
}

void CoogleIOTWebserver::handleRestart()
{
	webServer->send_P(200, "text/html", WEBPAGE_Restart);
}

void CoogleIOTWebserver::handleApiReset()
{
	iot->resetEEProm();
}

void CoogleIOTWebserver::handleApiRestart()
{
	iot->restartDevice();
}

void CoogleIOTWebserver::handleApiStatus()
{
  StaticJsonDocument<200> jsonBuffer;
  WiFiClientPrint<> p(webServer->client());

  JsonObject retval = jsonBuffer.to<JsonObject>();

  retval["status"] = !iot->_restarting;

  webServer->setContentLength(retval.size());
  webServer->send(200, "application/json", "");

  String pStr;
  serializeJson(retval,pStr);
	p.print(pStr);
  p.stop();
}
