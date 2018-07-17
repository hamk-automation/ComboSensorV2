/*
 Name:		HAMKComboSensor.ino
 Created:	2/22/2018 5:20:52 PM
 Author:	minhk
*/

#include "SoftwareSerial.h"
#include <FS.h> //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// needed for library
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <DNSServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <Wire.h>

#include <Adafruit_TSL2591.h>
#include <Adafruit_SGP30\Adafruit_SGP30.h>
#include <SCD30\SCD30.h>


#define DEBUG false
#define SENSORDEBUG 1
#define SERIAL_DEBUG 1
#define MQTT 1
Adafruit_TSL2591 tsl = Adafruit_TSL2591();
Adafruit_SGP30 sgp = Adafruit_SGP30();
SCD30 scd = SCD30();
SoftwareSerial PMS(D7, D8);

float t, rh, eco2, tvoc, lux, co2;

uint32_t lum;
uint16_t ir, full, visible;
long rssi;
String host = "iot.research.hamk.fi";
int httpReqCode = 0;
unsigned int pm10 = 0, pm25 = 0;
uint8_t sensorErrors = 0;
#define DAQ_INTERVAL 5
char sensorID[50] = "";
float difPressure;
// flag for saving data
bool shouldSaveConfig = true;
volatile bool measurementFlag = false, resetFlag = false;
int wifiStatus = 0;
int readResponse();
String mqttClientID;
Ticker DAQTimer, ResetTimer, TimeoutTimer;
bool timeoutFlag = false, configDone = false;

const int AutoSendOn[4] = { 0x68, 0x01, 0x40, 0x57 };
const int AutoSendOff[4] = { 0x68, 0x01, 0x20, 0x77 };
const int StartPmMeasure[4] = { 0x68, 0x01, 0x01, 0x96 };
const int StopPmMeasure[4] = { 0x68, 0x01, 0x02, 0x95 };
const int ReadPm[4] = { 0x68, 0x01, 0x04, 0x93 };

int isAutoSend = false;
int useReading = true;
boolean firstSetup = false;

WiFiClientSecure espClient;
PubSubClient mqttClient(host.c_str(), 8883, espClient);

// int pm25 = 0;   // PM2.5
// int pm10 = 0;   // PM10
// PubSubClient mqttClient(WiFi);
unsigned long lastReading = 0;
bool i2c_recovery(unsigned long timeout);
// callback notifying us of the need to save config
void saveConfigCallback() {
	Serial.println("Should save config");
	shouldSaveConfig = true;
}

void setup() {
	pinMode(D4, OUTPUT);
	digitalWrite(D4, LOW);
	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial.println();
	// clean FS, for testing
	//WiFi.disconnect();
	// SPIFFS.format();
	// read configuration from FS json
	WiFi.scanNetworksAsync(setupConfigPortal, true);
	setTimeout(30); // config timeout
	while (!timeoutFlag) {
		yield();
		if (configDone) break;
	}
	if (timeoutFlag && !configDone) ESP.restart();
	DAQTimer.attach(DAQ_INTERVAL, setMeasurementFlag);
}

void loop() {
	// put your main code here, to run repeatedly:
	mqttClient.loop();
	if (measurementFlag) {
		pushMqtt();
	}
	if (resetFlag) {
		readResponse();
		resetFlag = false;
		// Serial.print("PM 2.5: "); Serial.print(pm25);
		// Serial.print(" / PM 10: "); Serial.println(pm10);
	}
	// Serial.print("ram: "); Serial.println(ESP.getFreeHeap());
	yield();
}

void setMeasurementFlag() { measurementFlag = true; }

void setResetFlag() { resetFlag = true; }

void pushMqtt() {
	rssi = WiFi.RSSI();
	static char result_str[500] = "";
	printAllSensors();
	DynamicJsonBuffer jsonBuffer;
	JsonObject &json = jsonBuffer.createObject();
	json["id"] = sensorID;
	json["uptime"] = millis();
	json["rssi"] = rssi;
	json["t"] = t;
	json["rh"] = rh;
	json["lux"] = lux;
	json["co2"] = co2;
	json["tvoc"] = tvoc;
	json["eco2"] = eco2;
	json["pm2.5"] = pm25;
	json["pm10"] = pm10;
	json["ram"] = ESP.getFreeHeap();
	json.printTo(result_str);
#if SERIAL_DEBUG
	Serial.println(result_str);
#endif

#if SENSORDEBUG
	if ((!WiFi.isConnected()) || rssi < (-85)) {
		Serial.print("Trying to connect to saved wifi: ");
		Serial.println(WiFi.SSID());
		if (!WiFi.isConnected())
			WiFi.begin();
		else
			WiFi.reconnect();
		unsigned long timeout = millis();
		while (!WiFi.isConnected()) {
			// WiFi.begin();
			Serial.print("Connection status: ");
			Serial.println(WiFi.status());
			yield();
			delay(500);
			if (millis() - timeout >= 60000)
				break;
			// reset and try again, or maybe put it to deep sleep
		}
		if (!WiFi.isConnected())
			ESP.restart();
	}
#if MQTT
	uint8_t mqttRetries = 0;
	if (!mqttClient.connected()) {
		while ((!mqttClient.connected() || espClient.connected()) && mqttRetries <= 10) {
			mqttClient.connect(mqttClientID.c_str());
			Serial.println(mqttClient.state());
			if (mqttClient.connected()) {
				mqttClient.publish("hamk/testWifiSensor", result_str);
				break;
			}
			else {
				mqttRetries++;
			}
			mqttClient.loop();
		}
	}
	else {
		Serial.println(mqttClient.state());
		mqttClient.publish("hamk/testWifiSensor", result_str);
	}
#endif
	measurementFlag = false;
	return;
#endif
}

bool configureSensors(void) {
	int i = 0;
	unsigned long timeout = millis();
	Wire.begin(D2, D1);
	Wire.setClock(100000);
	Wire.setClockStretchLimit(200000);
	do {
		while (millis() - timeout <= 2000) {
			Serial.println(twi_status());
			yield();
		}
	} while (twi_status() != I2C_OK);
	if (!tsl.begin()) {
		Serial.println("Failed to start lux sensor");
		// return false;
	}
	tsl.setGain(TSL2591_GAIN_MED);                // 1x gain (bright light)
	tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS); // shortest integration time
												  // (bright light)
	if (!sgp.begin()) {

	}

	if (!scd.begin()) {

	}

	Wire.setClock(100000);
	Wire.setClockStretchLimit(200000);
	scd.setMeasurementInterval(2);
	scd.setTemperatureOffset(500);
	scd.trigContinuousMeasurement();
	/*
		PMS.begin(9600);
		PMS.enableRx(true);
		PMS.enableTx(true);
		PMS.println();
		Serial.println("Init PMS");
		sendCommand(StartPmMeasure);
		sendCommand(AutoSendOn);
		Serial.println("pms done");
	*/
	return true;
}

void sendCommand(const int *cmd) {
	int i;
	for (i = 0; i < 4; i++) {
		PMS.write(cmd[i]);
	}
	// let a unicorn pass
	delay(10);
}

int readResponse() {
	int i = 0;
	int buf[32];
	int l = 32;
	unsigned long start = millis();

	do {

		buf[i] = PMS.read(); // read bytes from device

		if (DEBUG) {
			Serial.print("i: ");
			Serial.print(i);
			Serial.print(" buf[i]: ");
			Serial.println(buf[i], HEX);
		}

		// check for HEAD or skip a byte
		if (i == 0 && !(buf[0] == 0x40 || buf[0] == 0x42 || buf[0] == 0xA5 ||
			buf[0] == 0x96)) {
			if (DEBUG) {
				Serial.println("Skipping Byte");
			}
			continue;
		}
		else {
			i++;
		}

		if (buf[0] == 0x42 && buf[1] == 0x4d) { // Autosend
			if (DEBUG) {
				Serial.println("Autosend");
			}
			l = 32;
		}

		if (buf[0] == 0x40 && buf[2] == 0x4) { // Reading
			if (DEBUG) {
				Serial.println("Reading");
			}
			l = 8;
		}

		if (buf[0] == 0xA5 && buf[1] == 0xA5) { // Pos. ACK
			if (DEBUG) {
				Serial.println("ACK");
			}
			return true;
		}

		if (buf[0] == 0x96 && buf[1] == 0x96) { // Neg. ACK
			if (DEBUG) {
				Serial.println("NACK");
			}
			return false;
		}

		if (millis() - start > 1000) { // trigger Timeout after 1 sec
			Serial.println("Timeout");
			return false;
		}

	} while (PMS.available() > 0 && i < l);

	// check checksum in Reading
	if (buf[2] == 0x04) {
		// HEAD+LEN+CMD
		int cs = buf[0] + buf[1] + buf[2];
		int c;

		// DATA
		for (c = 3; c < (2 + buf[1]); c++) {
			// Serial.println(buf[c]);
			cs += buf[c];
		}
		// CS = MOD((65536-(HEAD+LEN+CMD+DATA)), 256)
		cs = (65536 - cs) % 256;

		// validate checksum
		if (cs == buf[c]) {
			// calculate PM values
			pm25 = buf[3] * 256 + buf[4];
			pm10 = buf[5] * 256 + buf[6];
			return true;
		}
		else {
			Serial.println("Checksum mismatch");
		}
	}
	else if (buf[3] == 0x1c) { // Autoreading
		int cs = 0;
		int c;
		// DATA
		for (c = 0; c <= buf[3]; c++) {
			// Serial.println(buf[c]);
			cs += buf[c];
		}
		int checksum = buf[30] * 256 + buf[31];
		if (DEBUG) {
			Serial.print("Checksum: ");
			Serial.print(checksum, HEX);
			Serial.print(" CS: ");
			Serial.println(cs, HEX);
		}

		if (cs == checksum) {
			// calculate PM values
			pm25 = buf[6] * 256 + buf[7];
			pm10 = buf[8] * 256 + buf[9];
			return true;
		}
		else {
			if (DEBUG)
				Serial.println("Checksum mismatch");
		}
	}
	else {
		// unkownESP.getFreeHeap()
	}

	return false;
}

void printAllSensors() {
	lum = tsl.getFullLuminosity();
	ir = lum >> 16;
	full = lum & 0xFFFF;
	visible = full - ir;
	lux = tsl.calculateLux(full, ir);
	scd.readMeasurement();
	co2 = scd.fCO2;
	t = scd.fTemp;
	rh = scd.fRH;

	sgp.IAQmeasure();
	tvoc = sgp.TVOC;
	eco2 = sgp.eCO2;
	if (sensorErrors >= 10)
		ESP.restart();
}

inline float compute_dew_point_temp(float temperature,
	float humidity_percentage) {
	return pow(humidity_percentage / 100.0, 0.125) * (112.0 + 0.9 * temperature) +
		0.1 * temperature - 112.0;
}

// Compute the humidity given temperature and dew point temperature (temp in
// Celcius) Formula:
// http://www.ajdesigner.com/phphumidity/dewpoint_equation_relative_humidity.php
// f = 100 * ((112 - 0.1*T + Td) / (112 + 0.9 * T))^8
inline float compute_humidity_from_dewpoint(float temperature,
	float dew_temperature) {
	return 100.0 * pow((112.0 - 0.1 * temperature + dew_temperature) /
		(112.0 + 0.9 * temperature),
		8);
}

bool i2c_recovery(unsigned long timeout) {
	unsigned long t0 = millis();
	while (twi_status() != I2C_OK) {
		if (millis() - t0 >= timeout)
			return false;
		yield();
	}
	return true;
}

void setupConfigPortal(int networksFound) {
	TimeoutTimer.detach();
	Serial.println("mounting FS...");
	if (!configureSensors())
		ESP.restart();
	if (SPIFFS.begin()) {
		Serial.println("mounted file system");
		if (SPIFFS.exists("/config.json")) {
			// file exists, reading and loading
			Serial.println("reading config file");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile) {
				Serial.println("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);

				configFile.readBytes(buf.get(), size);
				DynamicJsonBuffer jsonBuffer;
				JsonObject &json = jsonBuffer.parseObject(buf.get());
				json.printTo(Serial);
				if (json.success()) {
					Serial.println("\nparsed json");
					strcpy(sensorID, json["sensorID"]);

				}
				else {
					Serial.println("failed to load json config");
				}
			}
		}
	}
	else {
		Serial.println("failed to mount FS");
	}
	mqttClientID = sensorID + '_' + String(ESP.getChipId());
	// end read
#if SENSORDEBUG
	bool oldWiFiInRange = false;
	if (WiFi.SSID() != "") {
		WiFi.scanNetworks(false, true);
		String preconfSSID = WiFi.SSID();
		timeoutFlag = false;
		TimeoutTimer.once(5, timeoutOccurred);
		while (WiFi.scanComplete() < 0 && !timeoutFlag) {
			yield();
		}
		if (timeoutFlag) {
		}
	}
	else {
		// The extra parameters to be configured (can be either global or just in
		// the setup) After connecting, parameter.getValue() will get you the
		// configured value id/name placeholder/prompt default length
		WiFiManagerParameter custom_sensorID("sensorID", "Location", sensorID, 32);
		// WiFiManager
		WiFiManager wifiManager;
		// wifiManager.resetSettings();
		wifiManager.setSaveConfigCallback(saveConfigCallback);
		wifiManager.addParameter(&custom_sensorID);
		wifiManager.setConfigPortalTimeout(60);
		// wifiManager.setConnectTimeout(60);
		wifiManager.setBreakAfterConfig(true);
		String networkID = "HAMK Sensor ";
		networkID += String(ESP.getChipId());
		if (!wifiManager.startConfigPortal("HAMK Combo Sensor")) {
			if (shouldSaveConfig) {
				strcpy(sensorID, custom_sensorID.getValue());
				Serial.println("saving config");
				DynamicJsonBuffer jsonBuffer;
				JsonObject &json = jsonBuffer.createObject();
				json["sensorID"] = sensorID;

				File configFile = SPIFFS.open("/config.json", "w");
				if (!configFile) {
					Serial.println("failed to open config file for writing");
				}

				json.printTo(Serial);
				json.printTo(configFile);
				configFile.close();
				// end save
			}
			digitalWrite(D4, HIGH);
			Serial.println("failed to connect and hit timeout");
			ESP.restart();
		}
		// connected
		Serial.println("connected...yeey :)");
		WiFi.printDiag(Serial);
		if (shouldSaveConfig) {
			strcpy(sensorID, custom_sensorID.getValue());
			Serial.println("saving config");
			DynamicJsonBuffer jsonBuffer;
			JsonObject &json = jsonBuffer.createObject();
			json["sensorID"] = sensorID;

			File configFile = SPIFFS.open("/config.json", "w");
			if (!configFile) {
				Serial.println("failed to open config file for writing");
			}

			json.printTo(Serial);
			json.printTo(configFile);
			configFile.close();
			// end save
		}
		ESP.restart();
	}
	if (oldWiFiInRange) {
		Serial.print("Trying to connect to saved wifi: ");
		Serial.println(WiFi.SSID());
		WiFi.mode(WIFI_STA);
		WiFi.begin();
		unsigned long timeout = millis();
		while (!WiFi.isConnected()) {
			wifiStatus = WiFi.status();
			Serial.print("Connection status: ");
			Serial.println(wifiStatus);
			delay(50);
			yield();
			// WiFi.begin();
			if (millis() - timeout >= 120000)
				break;
			// reset and try again, or maybe put it to deep sleep
		}
		if (!WiFi.isConnected()) {
			// The extra parameters to be configured (can be either global or just in
			// the setup) After connecting, parameter.getValue() will get you the
			// configured value id/name placeholder/prompt default length
			WiFiManagerParameter custom_sensorID("sensorID", "Location", sensorID,
				32);

			// WiFiManager
			WiFiManager wifiManager;
			// wifiManager.resetSettings();
			wifiManager.setSaveConfigCallback(saveConfigCallback);
			wifiManager.addParameter(&custom_sensorID);
			wifiManager.setConfigPortalTimeout(60);
			// wifiManager.setConnectTimeout(60);
			wifiManager.setBreakAfterConfig(true);
			String networkID = "HAMK Sensor ";
			networkID += String(ESP.getChipId());
			if (!wifiManager.startConfigPortal("HAMK Combo Sensor")) {
				if (shouldSaveConfig) {
					strcpy(sensorID, custom_sensorID.getValue());
					Serial.println("saving config");
					DynamicJsonBuffer jsonBuffer;
					JsonObject &json = jsonBuffer.createObject();
					json["sensorID"] = sensorID;

					File configFile = SPIFFS.open("/config.json", "w");
					if (!configFile) {
						Serial.println("failed to open config file for writing");
					}

					json.printTo(Serial);
					json.printTo(configFile);
					configFile.close();
					// end save
				}
				digitalWrite(D4, HIGH);
				Serial.println("failed to connect and hit timeout");
				ESP.restart();
			}
			// connected
			Serial.println("connected...yeey :)");
			if (shouldSaveConfig) {
				strcpy(sensorID, custom_sensorID.getValue());
				Serial.println("saving config");
				DynamicJsonBuffer jsonBuffer;
				JsonObject &json = jsonBuffer.createObject();
				json["sensorID"] = sensorID;

				File configFile = SPIFFS.open("/config.json", "w");
				if (!configFile) {
					Serial.println("failed to open config file for writing");
				}

				json.printTo(Serial);
				json.printTo(configFile);
				configFile.close();
				// end save
			}
			ESP.restart();
		}
		else {
			Serial.println("Connected to: ");
			WiFi.printDiag(Serial);
		}
	}
#endif
	Serial.println("local ip");
	Serial.println(WiFi.localIP());
	if (!espClient.connect(host, 8883)) {
		Serial.println("connection failed");
		configDone = false;
	}
	else {
		Serial.println("Connect to broker successful");
		configDone = true;
	}
}


void timeoutOccurred() {
	timeoutFlag = true;
}

void setTimeout(float seconds) {
	timeoutFlag = false;
	TimeoutTimer.once(seconds, timeoutOccurred);
}