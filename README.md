# CoogleIOT

A ESP8266 Library for easy IOT device development.

This based on the excellent [CoogleIOT](https://github.com/ThisSmartHouse/CoogleIOT) project.

The CoogleIOT library was created to make building IOT devices on the ESP8266 microcontroller easier by providing a
solid encapsulated framework for most of the common things you want to do on an IOT device, including:

- Captive Portal for configuration of the device - allowing you to configure the AP name, the Wifi Client, and the built in MQTT Client. Just connect to the AP and configure (mobile friendly).
- Built in persistent logging mechanisms using SPIFFS filesystem (also available for viewing from the web interface)
- Built in MQTT client (provided by Async MQTT client for ESP8266 and ESP32) 
- Built in UI libraries for the device (Mini.css for style, jquery 3.x for Javascript) that can be served from the AP
  using the `/css` or `/jquery` URLs
- Built in NTP client for access to local date / time on device
- Built in DNS Server during configuration for captive portal support when connected to the device as an AP directly
- Built in Security-minded tools like HTML Escaping and other filters to prevent malicious inputs
- Built in OTA firmware update support. Can both upload a new firmware from the UI or pull a new one down from a server
- Uses the WiFiMulti library to seemsly connect to the strongest signal when there is more than one AP for an SSID.

## Screenshots

WiFi Configuration

![Wifi Configuration](https://raw.github.com/coogle/CoogleIOT/master/screenshots/wifi-screen.png)

MQTT Client Configuration

![MQTT Configuration](https://raw.github.com/coogle/CoogleIOT/master/screenshots/mqtt-screen.png)

System Configuration

![System Configuration](https://raw.github.com/coogle/CoogleIOT/master/screenshots/system-screen.png)

Status Page

![Status Page](https://raw.github.com/coogle/CoogleIOT/master/screenshots/status-screen.png)

## Example

CoogleIOT is designed to hang out in the background so your sketches are focused on the things you actually want to
work on without having to worry about things like WiFi or MQTT clients. Here is an example of using it to control a
Garage Door w/open and close sensors:

```
/* GarageDoor-Opener.h */
#ifndef GARAGEDOOR_OPENER_H
#define GARAGEDOOR_OPENER_H

#define SERIAL_BAUD 115200

#define OPEN_SENSOR_PIN 2   // The pin that detects when the door is closed
#define CLOSE_SENSOR_PIN 5   // The pin that detects when the door is open
#define OPEN_SWTICH_PIN 14    // The pin that activates the open / close door action
#define LIGHT_SWITCH_PIN 4  // The pin that turns the light on / off

#define GARAGE_DOOR_STATUS_TOPIC "/status/garage-door"
#define GARAGE_DOOR_ACTION_TOPIC_DOOR "/garage-door/door"
#define GARAGE_DOOR_ACTION_TOPIC_LIGHT "/garage-door/light"
#define GARAGE_DOOR_MQTT_CLIENT_ID "garage-door"

#include <Arduino.h>

typedef enum {
    GD_OPEN,
    GD_CLOSED,
    GD_OPENING,
    GD_CLOSING,
    GD_UNKNOWN
} GarageDoorState;

#endif

```


```
#include <CoogleIOT.h>
#include "GarageDoor-Opener.h"

CoogleIOT* iot;

GarageDoorState _currentState = GD_UNKNOWN;

const char* getDoorStateString(GarageDoorState state)
{
	switch(state) {
		case GD_OPEN:
			return "open";
		case GD_CLOSED:
			return "closed";
		case GD_OPENING:
			return "opening";
		case GD_CLOSING:
			return "closing";
		case GD_UNKNOWN:
			return "unknown";
	}

	iot->warn("Garage Door State Value Unknown!");

	return "unknown";
}

GarageDoorState getGarageDoorState()
{
	bool isClosed, isOpen;
	GarageDoorState retval = GD_UNKNOWN;

	isOpen = digitalRead(OPEN_SENSOR_PIN) == LOW;
	isClosed = digitalRead(CLOSE_SENSOR_PIN) == LOW;


	if(isOpen && isClosed) {
		iot->error("Can't be both open and closed at the same time! Sensor failure!");

		retval = GD_UNKNOWN;
		return retval;
	}

	if(!isOpen && isClosed) {
		retval = GD_CLOSED;
		return retval;
	}

	if(isOpen && !isClosed) {
		retval = GD_OPEN;
		return retval;
	}

	if(!isOpen && !isClosed) {

		if((_currentState == GD_OPEN) || (_currentState == GD_CLOSING)) {
			retval = GD_CLOSING;
			return retval;
		}

		if((_currentState == GD_CLOSED) || (_currentState == GD_OPENING)) {
			retval = GD_OPENING;
			return retval;
		}
	}

	retval = GD_UNKNOWN;
	return retval;
}

void triggerDoor()
{
	iot->info("Triggering Garage Door Open");
	digitalWrite(OPEN_SWTICH_PIN, LOW);
	delay(200);
	digitalWrite(OPEN_SWTICH_PIN, HIGH);
}

void triggerLight()
{
	iot->info("Triggering Garage Door Light");
	digitalWrite(LIGHT_SWITCH_PIN, LOW);
	delay(200);
	digitalWrite(LIGHT_SWITCH_PIN, HIGH);

}

void setup()
{
	iot = new CoogleIOT(LED_BUILTIN);

	iot->enableSerial(SERIAL_BAUD)
     .MQTTClientId(GARAGE_DOOR_MQTT_CLIENT_ID, false)
		 .MQTTSubTopic(GARAGE_DOOR_ACTION_TOPIC_DOOR,
										[=](const char* topic, const char* payload,
												AsyncMqttClientMessageProperties properties) {
				iot->info("Handling Garage Door Action Request");
				iot->flashStatus(200, 1);
				triggerDoor();	
			 })
		 .MQTTSubTopic(GARAGE_DOOR_ACTION_TOPIC_LIGHT,
										[=](const char* topic, const char* payload,
												AsyncMqttClientMessageProperties properties) {
				iot->info("Handing Garage Door Light Request");
				iot->flashStatus(200, 2);
				triggerLight();	
			 })
	   .initialize();

  pinMode(OPEN_SWTICH_PIN, OUTPUT);
	pinMode(LIGHT_SWITCH_PIN, OUTPUT);
	pinMode(OPEN_SENSOR_PIN, INPUT_PULLUP);
	pinMode(CLOSE_SENSOR_PIN, INPUT_PULLUP);

	digitalWrite(OPEN_SWTICH_PIN, HIGH);
	digitalWrite(LIGHT_SWITCH_PIN, HIGH);

	if(iot->mqttActive()) {
    iot->MQTTQueueMessage(GARAGE_DOOR_STATUS_TOPIC,
                    			getDoorStateString(_currentState));

    iot->info("Garage Door Opener Initialized");

		} else {
			iot->error("MQTT Not initialized, Garage Door Opener Inactive");
	}
}

void loop()
{
	GarageDoorState liveState;

	iot->loop();

	if(iot->mqttActive()) {
		liveState = getGarageDoorState();

		if(liveState != _currentState) {
			iot->MQTTQueueMessage(GARAGE_DOOR_STATUS_TOPIC,
														getDoorStateString(liveState)); 
			_currentState = liveState;
    }
	}
}

```

There are other projects that use this library which serve as great examples of it's use as well. You should probably check out these:

[Coogle Switch](https://github.com/ThisSmartHouse/coogle-switch) - A CoogleIOT-powered ESP8266 sketch for creating smart switches that operate over MQTT (controlling a relay module of configured sized). Just set up which pins your relay operates on and it takes care of all the MQTT topics, etc. you need for it to work.

## Where's my Device?

When MQTT is enabled, CoogleIOT automatically sends a periodic heartbeat message to `/coogleiot/devices/<client_id>` containing a JSON payload with useful information:

```
{ 
    "timestamp" : "2017-10-27 05:27:13", 
    "ip" : "192.168.1.130", 
    "coogleiot_version" : "1.2.1", 
    "client_id" : "bbq-temp-probe" 
}
```

If running multiple CoogleIOT devices this can be very useful to keep track of them all by just subscribing to the `/coogleiot/devices/#` wildcard channel which will capture all the heartbeat transmissions.

## API

CoogleIOT is an evolving code base, so this API may change before this document is updated to reflect that. The best source is the source. When possible CoogleIOT uses a fluent interface, allowing you to chain method calls together:

```
  // Chaining method calls together
  iot->enableSerial(115200)
     ->initialize();
```

`void CoogleIOT::CoogleIOT(status_led_pin = NULL)` 
The library constructor. You may provide an optional pin to use for a status LED which will be used to indicate different
states the device can be in (i.e. WiFi initializing, etc.)

`bool CoogleIOT::initialize()`
Must be called in `setup()` of your sketch to initialize the library and it's components

`void CoogleIOT::loop()`
Must be called in `loop()` of your sketch

`CoogleIOT& CoogleIOT::enableSerial(int baud = 115200)`
Enables Serial output from the IOT library. Will initialize the `Serial` object for you at the baud rate specified if not
already initialized.

`AsyncMqttClient* CoogleIOT::getMQTTClient()`
Return a pointer to the built in Async MQTT client to use in your sketch

`CoogleIOT& MQTTSubTopic(const char* topic, mqttSubCallback callBack)`
Subscribe to MQTT topic and specify an callback function for messages received on that topic. The callback function
is defined as `void(const char* topic, const char* payload, AsyncMqttClientMessageProperties properties)`. The topic can
include wildcards.

`CoogleIOT& MQTTQueueMessage(const char* topic, const char* payload)`
Queue a message to the MQTT topic.

`CoogleIOT& MQTTQueueMessage(const char* topic , uint8_t QoS, bool retain, const char* payload)`
Queue a message to the MQTT topic specifying the QoS and retain properties.

`CoogleIOT& MQTTProcessQueue()`
Send queued MQTT messages to the broker. This is called by the `CoogleIOT::loop()` function and should not need to be
called otherwise.

`bool CoogleIOT::serialEnabled()`
Returns true if Serial is enabled

`CoogleIOT& CoogleIOT::flashStatus(speed_in_ms, repeat = 5)`
Flashes the defined pin / LED at a speed, repeating as defined (5 times by default)

`CoogleIOT& CoogleIOT::flashSOS()`
Flashes the status pin / LED in an SOS pattern (useful to indicate an error)

`CoogleIOT& CoogleIOT::resetEEProm()`
Resets the EEPROM memory used by CoogleIOT to NULL, effectively "factory resetting" the device

`void CoogleIOT::restartDevice()`
Restarts the device. Due to [This Bug](https://github.com/esp8266/Arduino/issues/1722), you must physically press the restart button on the ESP8266 after flashing via Serial. If you fail to do that, this command will hang the device.

`String CoogleIOT::filterAscii()`
Filters the provided string of anything that is not a printable ASCII character

`bool CoogleIOT::verifyFlashConfiguration()`
Verifies the Flash configuration for the device (what the device supports, vs. what the device is set as in your sketch) is
correct.

`CoogleIOT& CoogleIOT::syncNTPTime(int offsetSeconds, int daylightOffsetSeconds)`
Synchronizes and sets the local device date / time based on NTP servers. Must have a working WiFi connection to use this
method. The first parameter is the number of seconds local time is offset from UTC time (i.e. -5 hrs in seconds is America/New York). The second parameter is the number of seconds to offset based on daylight savings.

`String CoogleIOT::getWiFiStatus()`
Returns a string representing the current state of the WiFi Client

`bool CoogleIOT::mqttActive()`
Returns true/false indicating if the MQTT client is active and ready to use or not

`bool CoogleIOT::dnsActive()`
Returns true/false if the integrated captive portal DNS is enabled or not

`bool CoogleIOT::ntpActive()`
Returns true/false if the NTP client is online and synchronizing with NTP time servers

`bool CoogleIOT::firmwareClientActive()`
Returns true/false if the periodic firmware client (that will download a new firmware from a web server) is active or not. If active, the Firmware client will check every 30 minutes for a new firmware at the configured URL

`bool CoogleIOT::apStatus()`
Returns true/false if the AP of the device is active or not.

`CoogleIOT& CoogleIOT::registerTimer(int interval, callback)`
Create a callback timer that will call `callback` (void function with no params) every `interval` milliseconds. You can turn off the timer
by passing 0 as the interval. Useful for taking a sensor reading every X seconds, etc.

`void CoogleIOT::checkForFirmwareUpdate()`
Performs a check against the specified Firmware Server endpoint for a new version of this device's firmware. If a new version exists it performs the upgrade.

`bool writeConfig()`
Write the current state of the confguration to the EEProm. By setting the optional bool parameter on the setters to false it possible to update a number of setting and the make one write to save them to the EEProm.

The following getters/setters are pretty self explainatory. Each getter will return a `const char*` object of the value from EEPROM (or another primiative data type), with a matching setter. An optional bool parameter which defaults to true dictates
if the value is saved to the EEProm. :

`const char* CoogleIOT::RemoteAPName()`
`CoogleIOT& CoogleIOT::RemoteAPName(const char*, bool)`
`const char* CoogleIOT::RemoteAPPassword()`
`CoogleIOT& CoogleIOT::RemoteAPPassword(const char*, bool)`
`const char* CoogleIOT::MQTTHostname()`
`CoogleIOT& CoogleIOT::MQTTHostname(const char*, bool)`
`const char* CoogleIOT::MQTTUsername()`
`CoogleIOT& CoogleIOT::MQTTUsername(const char*, bool)`
`const char* CoogleIOT::MQTTPassword()`
`CoogleIOT& CoogleIOT::MQTTPassword(const char*, bool)`
`const char* CoogleIOT::MQTTClientId()`
`CoogleIOT& CoogleIOT::setMQTTClientId(const char*, bool)`
`int CoogleIOT::MQTTPort()`
`CoogleIOT& CoogleIOT::MQTTPort(int, bool)`
`const char* CoogleIOT::getAPName()`
`CoogleIOT& CoogleIOT::APName(const char*, bool)`
`const char* CoogleIOT::APPassword()`
`CoogleIOT& CoogleIOT::APPassword(const char*, bool)`
`const char* CoogleIOT::FirmwareUpdateUrl()`
`CoogleIOT& CoogleIOT::FirmwareUpdateUrl(const char*, bool)`

There are not saved to the EEProm.

`int MQTTQoS()`
`CoogleIOT& MQTTQoS(int)`
`bool MQTTRetain()`
`CoogleIOT& MQTTRetain(bool)`

## CoogleIOT Firmware Configuration

The Firmware default values and settings are defined in the `CoogleIOTConfig.h` file and can be overriden by providing new `#define` statements

`#define COOGLEIOT_STATUS_INIT 500`
Defines the status LED flash speed in MS for initial initialization

`#define COOGLEIOT_STATUS_WIFI_INIT 250`
Defines the status LED flash speed for WiFi initialization

`#define COOGLEIOT_STATUS_MQTT_INIT 100`
Defines the status LED flash speed for MQTT initialization

`#define COOGLEIOT_AP "COOGLEIOT_`
Defines the prepended string used for the default AP name. The remainder of the AP name will be a randomly generated number (i.e. COOGLEIOT_234934)

`#define COOGLEIOT_AP_DEFAULT_PASSWORD "coogleiot"`
The default AP password

`#define COOGLEIOT_DEFAULT_MQTT_CLIENT_ID "coogleIoT"`
The default MQTT client ID

`#define COOGLEIOT_DEFAULT_MQTT_PORT 1883`
The default MQTT Port

`#define COOGLEIOT_DEFAULT_MQTT_QOS 1`
The defaut MQTT QoS property

`#define COOGLEIOT_DEFAULT_MQTT_RETAIN true`
The default MQTT retain property

`#define COOGLEIOT_TIMEZONE_OFFSET ((3600 * 5) * -1)`
The default NTP Timezone offset (America/New York)

`#define COOGLEIOT_DAYLIGHT_OFFSET 0`
The default NTP Daylight Offset

`
#define COOGLEIOT_NTP_SERVER_1 "pool.ntp.org"
#define COOGLEIOT_NTP_SERVER_2 "time.nist.gov"
#define COOGLEIOT_NTP_SERVER_3 "time.google.com"
`
The three default NTP servers to attempt to synchronize with

`#define COOGLEIOT_FIRMWARE_UPDATE_CHECK_MS 54000000  // 15 Minutes in Milliseconds`
The frequency that we will check for a new Firmware Update if the server is configured. Defaults to 15 minutes.

`#define COOGLEIOT_DNS_PORT 53`
The default DNS port

`#define COOGLE_EEPROM_EEPROM_SIZE 1024`
The amount of EEPROM memory allocated to CoogleIOT, 1kb default. 

*IMPORTANT NOTE:* 
Do _NOT_ reduce this value below it's default value unless you really know what you are doing, otherwise you will break the firmware.

`#define COOGLEIOT_WEBSERVER_PORT 80`
The default Webserver port for the configuration system

`#define COOGLEIOT_DEBUG`
If defined, it will enable debugging mode for CoogleIOT which will dump lots of debugging data to the Serial port (if enabled)
