//crash problem

#include <WiFiClient.h>
#include "ThingsBoard.h"
#include <Arduino_MQTT_Client.h>
#include <ArduinoJson.h>

#define WIFI_AP_NAME            "Vodafonenet_Wifi_8047"
#define WIFI_PASSWORD       "U5DerrGGDUEz"

constexpr uint16_t MAX_MESSAGE_SIZE = 128;
constexpr uint16_t THINGSBOARD_PORT = 1883;

#define TOKEN               "pHYZ7VkKvoxTOyy6LDc1"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);


int status = true;

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}


void setup() {

  Serial.begin(115200);
  InitWiFi();
}


void loop() {
  delay(1000);

  // Reconnect to WiFi, if needed
  if ( WiFi.status() != WL_CONNECTED ) {
    reconnect();
    return;
  }

  if ( !tb.connected() ) {
    if ( !tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT) ) {
      Serial.println("Failed to connect to ThingsBoard Server...");
      return;
    }
  }

  tb.sendTelemetryData("Tempreature", 23.12);

  tb.loop();
}
