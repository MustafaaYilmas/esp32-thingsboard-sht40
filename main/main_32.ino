#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

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
int quant = 250;
int send_delay = 2000;
int send_passed = 0;

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

void setup() {
  Serial.begin(115200);
  InitWiFi();
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  ArduinoOTA.handle();
  delay(quant);
  send_passed += quant;

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

  // Generate random telemetry data
  float randomTemperature = random(0, 100);
  float randomHumidity = random(0, 100);
  float randomPressure = random(900, 1100);

  // Send telemetry data to ThingsBoard
  tb.sendTelemetryData("temperature", randomTemperature);
  tb.sendTelemetryData("humidity", randomHumidity);
  tb.sendTelemetryData("pressure", randomPressure);

  Serial.println("Telemetry data sent.");

  if (send_passed > send_delay) {
    send_passed = 0;
  }

  // Process messages
  tb.loop();
}
