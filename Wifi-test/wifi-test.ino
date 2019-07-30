/*
    This sketch establishes a TCP connection to a "quote of the day" service.
    It sends a "hello" message, and then prints received data.
*/

#include <ESP8266WiFi.h>
//BH1750 (Lux)
#include <BH1750FVI.h>
#include <Wire.h>

#ifndef STASSID
#define STASSID ""
#define STAPSK  ""
#endif

BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);

//wifi settings
const char* ssid     = STASSID;
const char* password = STAPSK;

//site to test connectivity
const char* host = "djxmmx.net";
const uint16_t port = 17;

//defining variable types
int moisture_level;
uint16_t lux_level;
//unsigned long myChannelNumber = ;  // Replace the 0 with your channel number
//const char * myWriteAPIKey = "";    // Paste your ThingSpeak Write API Key between the quotes
unsigned long delayTime;

//BH1750 reading function
void light_sensor() {
  //initializes wire library
  Wire.begin();
  //initializes the light sensor in ONE_TIME_HIGH_RES_MODE
  LightSensor.begin();
  //reads the light value and then it should shut down
  lux_level = LightSensor.GetLightIntensity();
  //prints the value to serial console
  Serial.print("Light: ");
  Serial.println(lux_level);
}

void setup() {

  //make sure the wifi is off on the first boot
  //not sure how effective this is as it only makes a difference on a cold boot
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  //initializes serial output
  Serial.begin(115200);

  //debug output displaying the wifi is disabled
  Serial.println();
  Serial.println("Wifi disabled");
  delay(5000);

  //time to read the light value and store it
  Serial.println();
  Serial.println("Reading Light value");
  light_sensor();
  delay(5000);  

  //now that we have read the light value we can upload it
  //to do this, we must turn on the wifi.
  Serial.print("Turining on wifi");
  WiFi.forceSleepWake();
  delay(5000);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    delay(5000);
    return;
  }

  // This will send a string to the server
  Serial.println("sending data to server");
  if (client.connected()) {
    client.println("hello from ESP8266");
  }

  // wait for data to be available
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      delay(60000);
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  Serial.println("receiving from remote server");
  // not testing 'client.connected()' since we do not need to send data here
  while (client.available()) {
    char ch = static_cast<char>(client.read());
    Serial.print(ch);
  }

  // Close the connection
  Serial.println();
  Serial.println("closing connection");
  client.stop();
  WiFi.disconnect(true);
  delay(1);

  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
  Serial.println("I'm awake, but I'm going into deep sleep mode for 15 seconds");
  ESP.deepSleep(15e6, WAKE_RF_DISABLED);
}