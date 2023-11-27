#ifdef ESP8266
#include <ESP8266WiFi.h>
// Disable PROGMEM because the ESP8266WiFi library,
// does not support flash strings.
#define THINGSBOARD_ENABLE_PROGMEM 0
#endif


// Sending data can either be done over MQTT and the PubSubClient
// or HTTPS and the HTTPClient, when using the ESP32 or ESP8266
#define USING_HTTPS false

// Whether the given script is using encryption or not,
// generally recommended as it increases security (communication with the server is not in clear text anymore),
// it does come with an overhead tough as having an encrypted session requires a lot of memory,
// which might not be avaialable on lower end devices.
#define ENCRYPTED false

// Enables sending messages that are bigger than the predefined message size,
// where the message will be sent byte by byte as a fallback instead.
// Requires an additional library, see https://github.com/bblanchon/ArduinoStreamUtils for more information.
#define THINGSBOARD_ENABLE_STREAM_UTILS 1

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>


constexpr char WIFI_SSID[] = "Vodafonenet_Wifi_8047";
constexpr char WIFI_PASSWORD[] = "U5DerrGGDUEz";

constexpr char TOKEN[] = "pHYZ7VkKvoxTOyy6LDc1";


constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;


// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SIZE = 128U;

// Baud rate for the debugging serial connection
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;


//keys
constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";



// Initialize underlying client, used to establish a connection

WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);



/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");

}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void setup() {
  // If analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  //randomSeed(analogRead(0));
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
}

void loop() {
  delay(1000);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  Serial.println("Sending temperature data...");
  tb.sendTelemetryData(TEMPERATURE_KEY, random(10, 31));


  Serial.println("Sending humidity data...");
  tb.sendTelemetryData(HUMIDITY_KEY, random(40, 90));

  tb.loop();

}
