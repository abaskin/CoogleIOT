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

CoogleIOTWebserver::CoogleIOTWebserver(CoogleIOT& _iot)
{
	CoogleIOTWebserver(_iot, 80);
}

CoogleIOTWebserver::CoogleIOTWebserver(CoogleIOT& _iot, int port)
{
  this->iot = &_iot;
  this->serverPort = port;

  iot->info(F("Creating Configuration Web Server"));
  this->webServer =
      std::unique_ptr<ESP8266WebServer>(new ESP8266WebServer(this->serverPort));
}

CoogleIOTWebserver::~CoogleIOTWebserver() { }

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

bool CoogleIOTWebserver::initialize()
{
	iot->info(F("Initializing Webserver"));

	initializePages();
	webServer->begin();

	iot->info(F("Webserver Initiailized!"));

	return true;
}

void CoogleIOTWebserver::loop()
{
	webServer->handleClient();
}

String CoogleIOTWebserver::htmlEncode(String input) const
{
	input.replace(F("&"), "&amp;");
	input.replace(F("<"), "&lt;");
	input.replace(F(">"), "&gt;");
	input.replace(F("\""), "&quot;");
	input.replace(F("\'"), "&#39;");
	return input;
}

void CoogleIOTWebserver::handleRoot()
{
	String page(FPSTR(WEBPAGE_Home));

	page.replace(F("{{ap_name}}"), htmlEncode(iot->APName()));
	page.replace(F("{{ap_password}}"), htmlEncode(iot->APPassword()));
	page.replace(F("{{remote_ap_name}}"), htmlEncode(iot->SSIDName()));
	page.replace(F("{{remote_ap_password}}"), htmlEncode(iot->SSIDPassword()));
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
	page.replace(F("{{mqtt_status}}"), iot->mqttActive() ? F("Active") : F("Not Connected"));
	page.replace(F("{{ntp_status}}"), iot->timeSet() ? F("Active") : F("Not Connected"));
	page.replace(F("{{dns_status}}"), iot->dnsActive() ? F("Active") : F("Disabled"));
	page.replace(F("{{firmware_update_status}}"), iot->firmwareClientActive() ? F("Active") : F("Disabled"));
	page.replace(F("{{coogleiot_ap_status}}"), iot->apStatus() ? F("Active") : F("Disabled"));

  webServer->send(200, "text/html", page.c_str());
}

void CoogleIOTWebserver::handleJS()
{
	webServer->sendHeader("Content-Encoding", "gzip", true);
	webServer->send_P(200, PSTR("application/javascript"), jquery_3_2_1_min_js_gz, jquery_3_2_1_min_js_gz_len);
}

void CoogleIOTWebserver::handleCSS()
{
	webServer->send_P(200, PSTR("text/css"), WEBPAGE_CSS, mini_default_min_css_len);
}

void CoogleIOTWebserver::handle404()
{
	webServer->send_P(404, PSTR("text/html"), WEBPAGE_NOTFOUND);
}

void CoogleIOTWebserver::handleLogs()
{
	auto logFile = iot->LogFile();

	logFile.seek(0, SeekSet);
	webServer->streamFile(logFile, "text/html");
	logFile.seek(0, SeekEnd);
}

void CoogleIOTWebserver::handleFirmwareUploadResponse()
{
	if(_manualFirmwareUpdateSuccess) {
		webServer->send_P(200, PSTR("text/html"), WEBPAGE_Restart);
		return;
	}

	webServer->send_P(200, PSTR("text/html"),
										PSTR("There was an error uploading the firmware"));
}

void CoogleIOTWebserver::handleFirmwareUpload()
{
	HTTPUpload& upload = webServer->upload();
	uint32_t maxSketchSpace;

	switch(upload.status) {
		case UPLOAD_FILE_START:
			WiFiUDP::stopAll();

			iot->info(F("Receiving Firmware Upload..."));

			maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

			if(!Update.begin(maxSketchSpace)) {
				iot->error(F("Failed to begin Firmware Upload!"));

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}

			break;
		case UPLOAD_FILE_WRITE:

			if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {

				iot->error(F("Failed to write Firmware Upload!"));

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}
			break;

		case UPLOAD_FILE_END:

			if(Update.end(true)) {

				iot->info(F("Firmware updated!"));

				_manualFirmwareUpdateSuccess = true;

			} else {
				iot->error(F("Failed to update Firmware!"));

				if(iot->serialEnabled()) {
					Update.printError(Serial);
				}

				_manualFirmwareUpdateSuccess = false;
			}

			break;

		case UPLOAD_FILE_ABORTED:
			Update.end();

			iot->info(F("Firmware upload aborted!"));

			_manualFirmwareUpdateSuccess = false;

			break;
	}

	yield();
}

void CoogleIOTWebserver::handleSubmit()
{
  const char* retval = R"({"status":true})";

	iot->APName(webServer->arg("ap_name").c_str(),false);
	iot->APPassword(webServer->arg("ap_password").c_str(), false);
	iot->SSIDName(webServer->arg("remote_ap_name").c_str(),false);
	iot->SSIDPassword(webServer->arg("remote_ap_password").c_str(),false);
	iot->MQTTHostname(webServer->arg("mqtt_host").c_str(),false);
	iot->MQTTPort(webServer->arg("mqtt_port").toInt(),false);
	iot->MQTTUsername(webServer->arg("mqtt_username").c_str(),false);
	iot->MQTTPassword(webServer->arg("mqtt_password").c_str(),false);
	iot->MQTTClientId(webServer->arg("mqtt_client_id").c_str(),false);
	iot->MQTTLWTTopic(webServer->arg("mqtt_lwt_topic").c_str(),false);
	iot->MQTTLWTMessage(webServer->arg("mqtt_lwt_message").c_str(),false);
	iot->FirmwareUpdateUrl(webServer->arg("firmware_url").c_str(),false);

	iot->writeConfig();

	webServer->setContentLength(strlen(retval));
	webServer->send(200, "application/json", "");

  WiFiClientPrint<> p(webServer->client());
	p.print(retval);
	p.stop();
}

void CoogleIOTWebserver::handleReset()
{
	webServer->send_P(200, PSTR("text/html"), WEBPAGE_Restart);
	iot->resetEEProm();
}

void CoogleIOTWebserver::handleRestart()
{
	webServer->send_P(200, PSTR("text/html"), WEBPAGE_Restart);
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
  StaticJsonDocument<200> retval;
  WiFiClientPrint<> p(webServer->client());

  retval["status"] = !iot->_restarting;

  webServer->setContentLength(measureJson(retval));
  webServer->send_P(200, PSTR("application/json"), PSTR(""));

  p.print(retval.as<const char*>());
  p.stop();
}
