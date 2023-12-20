#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "Adafruit_SHT4x.h"

// WiFi network credentials
#define WIFI_AP_NAME        "Vodafonenet_Wifi_8047"
#define WIFI_PASSWORD       "U5DerrGGDUEz"

// Credentials for the NDU platform
#define TOKEN       "It1qTkLliajffjEAGUeW"
#define NDU_SERVER  "smartapp.netcad.com"
constexpr uint16_t NDU_PORT = 1883;
constexpr uint16_t MAX_MESSAGE_SIZE = 128; 

// Defining objects necessary for MQTT and NDU on ESP32
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

int status = WL_IDLE_STATUS;

// Creating an instance of the Adafruit SHT4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// Structure to hold data from the SHT4x sensor
struct SHT40Data
{
  float temperature;
  float humidity;
};

// Function to initialize WiFi connection
void InitWiFi()
{
  // Connecting to the WiFi network
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

// Function to reconnect to WiFi if the connection is lost
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

// Initializing the SHT4x sensor
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

// Function to read temperature and humidity from the SHT4x sensor
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
  delay(5000);
  
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }

  if (!tb.connected()) {
    if (!tb.connect(NDU_SERVER, TOKEN, NDU_PORT)) {
      Serial.println("Failed to connect to NDU Server...");
      return;
    }
  }

  SHT40Data returnedData = readTemperatureAndHumidity();

  // Sending temperature and humidity data to NDU
  tb.sendTelemetryData("developer_temperature", returnedData.temperature);
  tb.sendTelemetryData("developer_humidity", returnedData.humidity);

  //Serial.println("Telemetry data sent.");
  tb.loop();
}
