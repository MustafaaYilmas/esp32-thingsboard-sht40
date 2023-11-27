# Monitoring SHT4x Sensor Data with ESP32 and ThingsBoard

This project involves an IoT application that uses ESP32 and the ThingsBoard platform to collect data from an SHT4x temperature and humidity sensor and send this data to a ThingsBoard server.

## Required Libraries

```cpp
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "Adafruit_SHT4x.h"
```
This section includes the libraries required for the project. `ThingsBoard.h`, `Arduino_MQTT_Client.h`, and `WiFiClient.h` are necessary for connecting your IoT device to the ThingsBoard server over the internet. `ArduinoJson.h` is used for data processing and MQTT messages. `Adafruit_SHT4x.h` is the library required to interact with the SHT4x sensor.

## Settings and Constants

```cpp
#define WIFI_AP_NAME        "Vodafonenet_Wifi_8047"
#define WIFI_PASSWORD       "U5DerrGGDUEz"
#define TOKEN               "pHYZ7VkKvoxTOyy6LDc1"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

constexpr uint16_t MAX_MESSAGE_SIZE = 128;
constexpr uint16_t THINGSBOARD_PORT = 1883;
```
This section defines settings like the name and password of the WiFi network, and the ThingsBoard server and connection port.

## Global Variables

```cpp
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
int status = WL_IDLE_STATUS;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
```
Here, variables required for the WiFi and MQTT client, ThingsBoard object, and SHT4x sensor are defined.

## Helper Functions and Structures

Functions like `InitWiFi`, `reconnect`, `shtBegin`, and `readTemperatureAndHumidity` are used for initiating WiFi connection, reconnecting, initializing the SHT4x sensor, and reading temperature and humidity data from the sensor, respectively. The `SHT40Data` structure is defined to store temperature and humidity data.

## Main Functions

`setup` and `loop` are the main functions. The `setup` function runs once when the program starts and does the necessary initial setups. The `loop` function runs continuously and reads sensor data and sends it to ThingsBoard.


