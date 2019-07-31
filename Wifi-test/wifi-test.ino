#include <ESP8266WiFi.h>
//BH1750 (Lux)
#include <BH1750FVI.h>
#include <Wire.h>
//BME280 (Temp.Hum.,Press.)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//MAX17403 (Battery)
#include <MAX17043.h>

#ifndef STASSID
#define STASSID ""
#define STAPSK  ""
#endif

// MAX17043 fuelGauge(20,4); //alert threshold, alert pin - not needed.
BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);
Adafruit_BME280 bme;

//wifi settings
const char* ssid     = STASSID;
const char* password = STAPSK;

//TESTING CODE
//site to test connectivity
const char* host = "djxmmx.net";
const uint16_t port = 17;

//min return value of the moisture sensor to confirm it's working.
const int min_moisture_level = 200;
const int desired_moisture_level = 700;
const double SLEEP_TIME = 15e6;

//define sea level Pressure
//SEALEVELPRESSURE_HPA = (get fom open weather map)

//defining variable types
int moisture_level;
uint16_t lux_level;
//unsigned long myChannelNumber = ; // Replace the 0 with your channel number
//const char * myWriteAPIKey = "";  // Paste your ThingSpeak Write API Key between the quotes
//unsigned long delayTime;          //don't need this.

int Moisture_Pin = A0; // Sets the Analog input to A0
int Config_Pin = D3;  // Config button connected to digital pin D3
int Motor_Pin = D4;    // Motor enable circuit connected to digital pin D4
int Moisture_Power_Pin = D5 // Pin to power moisture sensor so that it's not on all of the time

void define_pins() {
  pinMode(Motor_Pin, OUTPUT);  // sets the digital pin D4 as output
  digitalWrite(Motor_Pin, LOW) // Sets the input state to GND
  pinMode(Config_Pin, INPUT);  // sets the digital pin D3 as input
  digitalWrite(Config_Pin, LOW) // Sets the input state to GND
  pinMode(Moisture_Pin, INPUT);  // sets the Analog pin A0 as input
  pinMode(Moisture_Power_Pin, OUTPUT);  // sets the digital pin D5 as output
  digitalWrite(Moisture_Power_Pin, LOW) // Sets the input state to GND
}

//BH1750 reading function
void light_sensor() {
  Serial.println();
  Serial.println("Reading Light value");
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

//Do we need to water the plant?
void water_plant() {
 	if (moisture_level < desired_moisture_level) {
  	Serial.print("Watering plant");
    digitalWrite(Motor_Pin, HIGH)
 		while (moisture_level < desired_moisture_level) do {
			moisture_level = analogRead(Moisture_Pin);  // read the input pin
		}    
 	  digitalWrite(Motor_Pin, LOW)
    Serial.print("Plant watered");
  }
}

//Moisture reading function
void moisture_sensor() {
  Serial.println();
  Serial.println("Reading Moisture value");
  //wake up sensor
  digitalWrite(Moisture_Power_Pin, HIGH) // Sets the input state to 3.3V
  //read sensor values
  moisture_level = analogRead(Moisture_Pin);  // read the input pin

  /* if the moisture sensor fails, we don't want the motor turning on all
  of the time. So we check against a minimum set value.
  If that threshold is missed, we set the default moisture level to the max. */

  Serial.print("Moisture: ");
  if (moisture_level < min_moisture_level) {
    moisture_level = 1023;
    Serial.println("ERROR");
    }
  else {
    water_plant();
    Serial.println(moisture_level); // Print the current moisture value
  }
  
  //sleep the sensor
  digitalWrite(Moisture_Power_Pin, LOW) // Sets the input state to GND
}

//MAX17043 reading function
void battery_sensor() {
  //initializes the battery sensor
  fuelGauge.begin();
  //wake up sensor
  Serial.println("Waking...");
  fuelGauge.wake();
  Serial.println();
  Serial.println("Reading battery capacity and percentage values");
  //read sensor values
  Serial.print("Version: ");
  Serial.println(fuelGauge.getVersion());
  Serial.print("Alert Threshold: ");
  Serial.println(fuelGauge.getAlertThreshold());
  Serial.print("Alert Threshold Register Version: ");
  Serial.println(fuelGauge.getAlertThresholdRegister());
  Serial.print("Battery Voltage: ");
  Serial.println(fuelGauge.getBatteryVoltage());
  Serial.print("Battery Percentage: ");
  Serial.println(fuelGauge.getBatteryPercentage());
  Serial.print("Is Alerting? ");
  Serial.println(fuelGauge.isAlerting());
  //sleep sensor
  Serial.println("Sleeping...");
  fuelGauge.sleep();
}

//BME280 reading function
void bme_sensor(){
  Serial.println();
  Serial.println("Reading Temp,Humidity and Pressure values");

  //initializes the BME280 sensor in normal mode.
  bme.begin();

  // Standard monitoring mode
  // Serial.println("normal mode, 16x oversampling for all, filter off,");
  // Serial.println("0.5ms standby period");

  // weather monitoring
  // Serial.println("-- Weather Station Scenario --");
  // Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  // Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
                      
  // suggested rate is 1/60Hz (1m)
 
  /* indoor navigation
  Serial.println("-- Indoor Navigation Scenario --");
  Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
  Serial.println("0.5ms standby period, filter 16x");
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,  // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1,  // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5 );
    
  // suggested rate is 25Hz
  // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
  // T_ovs = 2
  // P_ovs = 16
  // H_ovs = 1
  // = 40ms (25Hz)
  // with standby time that should really be 24.16913... Hz
  delayTime = 41;

  //Suggestions taken from here:
  //https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
  */

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  // power down BME280 - handled in forced mode. N/A in normal mode.

}

void enable_wifi() {
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
    delay(100);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void disable_wifi_on_boot() {
  //make sure the wifi is off on the first boot
  //not sure how effective this is as it only makes a difference on a cold boot
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  Serial.println();
  Serial.println("Wifi disabled");
}

void wifi_connectivity_test() {
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
}

void deep_sleep() {
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when it wakes up
  Serial.println("I'm awake, but I'm going into deep sleep mode for 15 seconds");
  ESP.deepSleep(SLEEP_TIME, WAKE_RF_DISABLED);
}

void setup() {

  //initializes serial output
  Serial.begin(115200);
  define_pins();

  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  //debug output displaying the wifi is disabled
  disable_wifi_on_boot();
  delay(5000);

  //time to read the moisture value and store it
  moisture_sensor();
  delay(5000);  

  //time to read the light value and store it
  light_sensor();
  delay(5000); 

  //time to read the bme values and store them
  bme_sensor();
  delay(5000);  

  //time to read the battery values and store them
  battery_sensor();
  delay(5000);

  //now that we have read the sensor values we can upload it
  //to do this, we must turn on the wifi.
  enable_wifi();

  wifi_connectivity_test();
  deep_sleep();
}

void loop() {}