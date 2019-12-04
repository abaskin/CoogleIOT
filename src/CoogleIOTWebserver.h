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

#ifndef COOGLEIOT_WEBSERVER_H
#define COOGLEIOT_WEBSERVER_H

#include <ESP8266WebServer.h>
#include "CoogleIOT.h"
#include "CoogleIOTConfig.h"
#include "DNSServer/DNSServer.h"
#include "WiFiClientPrint.h"

#include <memory>

#include "webpages/home.h"
#include "webpages/mini_css_default.h"
#include "webpages/jquery-3.2.1.min.h"
#include "webpages/404.h"
#include "webpages/restarting.h"

class CoogleIOT;

class CoogleIOTWebserver
{
	public:
		CoogleIOTWebserver(CoogleIOT& _iot);
		CoogleIOTWebserver(CoogleIOT& _iot, int port);
		~CoogleIOTWebserver();

		String htmlEncode(String) const;

		bool initialize();
		void handleRoot();
		void handle404();
		void handleCSS();
		void handleJS();
		void handleSubmit();
		void handleReset();
		void handleRestart();
		void handleFirmwareUpload();
		void handleFirmwareUploadResponse();
		void handleLogs();

		void handleApiStatus();
		void handleApiReset();
		void handleApiRestart();

		void loop();
	protected:
		CoogleIOTWebserver& initializePages();
	private:
		std::unique_ptr<ESP8266WebServer> webServer;
		CoogleIOT* iot;
		bool _manualFirmwareUpdateSuccess = false;
		int serverPort = 80;
};

#endif
