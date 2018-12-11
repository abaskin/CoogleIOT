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
