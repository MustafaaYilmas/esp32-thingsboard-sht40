#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "Adafruit_SHT4x.h"

#define WIFI_AP_NAME        "Vodafonenet_Wifi_8047"
#define WIFI_PASSWORD       "U5DerrGGDUEz"
#define TOKEN               "pHYZ7VkKvoxTOyy6LDc1"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

constexpr uint16_t MAX_MESSAGE_SIZE = 128;
constexpr uint16_t THINGSBOARD_PORT = 1883;


WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
int status = WL_IDLE_STATUS;


Adafruit_SHT4x sht4 = Adafruit_SHT4x();


struct SHT40Data
{
  float temperature;
  float humidity;
};


void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  status = WiFi.status();
  if (status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

void shtBegin(){
    Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }

  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
}

SHT40Data readTemperatureAndHumidity(){
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  SHT40Data data;
  data.temperature = temp.temperature;
  data.humidity = humidity.relative_humidity;
  return data;
}


void setup() {
  Serial.begin(115200);
  InitWiFi();
  delay(1000);
  shtBegin();
}

void loop() {
  delay(1000);
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }

  if (!tb.connected()) {
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect to ThingsBoard Server...");
      return;
    }
  }

  SHT40Data returnedData = readTemperatureAndHumidity();

  tb.sendTelemetryData("temperature", returnedData.temperature);
  tb.sendTelemetryData("humidity", returnedData.humidity);

  Serial.println("Telemetry data sent.");

  // Process messages
  tb.loop();
}
