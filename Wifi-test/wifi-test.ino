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
const int port = 17;

//-----------------------------------------------------------------
// in the future, I would like for these settings to be set via webgui.
//-----------------------------------------------------------------

//min return value of the moisture sensor to confirm it's working.
const int min_moisture_level = 20;
const int desired_moisture_level = 700;
const double SLEEP_TIME = 15e6;
const double DELAY_TIME = 5000;

//define sea level Pressure
int SEALEVELPRESSURE_HPA = 1010;

//defining variable types
int moisture_level;
int lux_level;
boolean config_mode_enabled;
//unsigned long myChannelNumber = ; // Replace the 0 with your channel number
//const char * myWriteAPIKey = "";  // Paste your ThingSpeak Write API Key between the quotes
//unsigned long delayTime;          //don't need this.

//ThingSpeak Twitter setup.
//-----------------------------------------------------------------
// ThingSpeak Settings
char thingSpeakAddress[] = "api.thingspeak.com";
String thingtweetAPIKey = "XXXMX2WYYR0EV68M";
// Variable Setup
long lastConnectionTime = 0; 
boolean lastConnected = false;
int failedCounter = 0;
//-----------------------------------------------------------------

int Moisture_Pin = A0; // Sets the Moitsure input to A0
int Moisture_Power_Pin = D3; // Pin to power moisture sensor so that it's not on all of the time
int built_in_led = D4;    // Do not use this port for output as it enables the built-in led
int Motor_Pin = D5;    // Motor enable circuit connected to digital pin D5
int Config_Pin = D6;  // Configuration button connected to digital pin D6

void define_pins() {
  pinMode(Motor_Pin, OUTPUT);  // sets the digital pin D5 as output
  digitalWrite(Motor_Pin, LOW); // Sets the input state to GND
  pinMode(Config_Pin, INPUT);  // sets the digital pin D3 as input
  digitalWrite(Config_Pin, LOW); // Sets the input state to GND
  pinMode(Moisture_Pin, INPUT);  // sets the Analog pin A0 as input
  pinMode(Moisture_Power_Pin, OUTPUT);  // sets the digital pin D5 as output
  digitalWrite(Moisture_Power_Pin, LOW); // Sets the input state to GND
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
  if (moisture_level < min_moisture_level) { // sensor broken or disconnected
    moisture_level = 1023; //return with the highest value to stop watering
    Serial.println("Error with sensor");
    }
  else {
  if (moisture_level < desired_moisture_level) {
  	Serial.print("Watering plant");
    digitalWrite(Motor_Pin, HIGH);
 		while (moisture_level < desired_moisture_level) {
			moisture_level = analogRead(Moisture_Pin);  // read the input pin
      Serial.println(moisture_level);
      //yield();
      delay(500);
		}    
 	  digitalWrite(Motor_Pin, LOW);
    Serial.print("Plant watered");
    Serial.print("New Moisture: ");
    Serial.println(moisture_level); // Print the new moisture value
    }
  }
}

//Moisture reading function
void moisture_sensor() {

  Serial.println();
  Serial.println("Reading Moisture value");

  //wake up sensor
  digitalWrite(Moisture_Power_Pin, HIGH); // Sets the input state to 3.3V

  //read sensor values
  moisture_level = analogRead(Moisture_Pin);  // read the input pin

  Serial.print("Moisture: ");
  Serial.println(moisture_level); // Print the current moisture value

  water_plant();

  //sleep the sensor
  digitalWrite(Moisture_Power_Pin, LOW); // Sets the input state to GND
}

//MAX17043 reading function
void battery_sensor() {
  //initializes the battery sensor
  FuelGauge.begin();
  //wake up sensor
  Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  Serial.println("Waking FuelGuage");
  if (FuelGauge.isSleeping()){
    FuelGauge.wake();
    if (!FuelGauge.isSleeping()){
      Serial.println("Fuel Gauge is now awake.");
    }
    else {
      Serial.println("Failed to wake Fuel Gauge.");
    }
  }
  else {
    Serial.println("Fuel Gauge is already awake.");
  }
  Serial.println();
  Serial.println("Reading battery capacity and percentage values");
  //read sensor values
  Serial.print("Version:   "); Serial.println(FuelGauge.version());
  Serial.print("ADC:   "); Serial.println(FuelGauge.adc());
  Serial.print("Voltage:   "); Serial.print(FuelGauge.voltage()); Serial.println(" v");
  Serial.print("Percent:   "); Serial.print(FuelGauge.percent()); Serial.println("%");
  Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  Serial.print("Alert: "); Serial.println(FuelGauge.alertIsActive() ? "Yes" : "No");
  Serial.print("Threshold: "); Serial.println(FuelGauge.getThreshold());
  Serial.print("Compensation:  0x"); Serial.println(FuelGauge.compensation(), HEX);
  //sleep sensor
  Serial.println("Sleeping FuelGuage");
  if (!FuelGauge.isSleeping()){
    FuelGauge.sleep();
    if (FuelGauge.isSleeping()){
      Serial.println("Fuel Gauge put in sleep mode.");
      }
    else {
      Serial.println("Fuel Gauge failed to be put in sleep mode.");
      }
    }
  else {
  Serial.println("Fuel Gauge is already in sleep mode.");
  }
  Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  //https://github.com/porrey/max1704x
}

//BME280 reading function
void bme_sensor(){
  Serial.println();
  Serial.println("Reading Temp,Humidity and Pressure values");

  //initializes the BME280 sensor in normal mode.
  bme.begin();

  // weather monitoring
  // Serial.println("-- Weather Station Scenario --");
  // Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  // Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

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
  delay(DELAY_TIME);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    if (i=> 10 ) {exit;}
    else {    
    delay(500);
    Serial.print(".");
    i++
    }
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

void upload_sensor_reading()
{
  ThingSpeak.begin(client); 
  int x = ThingSpeak.writeField(myChannelNumber, 1, lux, myWriteAPIKey);
  
  // Check the return code
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}

void deep_sleep() {
  WiFi.disconnect(true);
  delay(1);
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when it wakes up
  Serial.println("I'm awake, but I'm going into deep sleep mode for 15 seconds");
  ESP.deepSleep(SLEEP_TIME, WAKE_RF_DISABLED);
}

void config_mode() {
  while (Config_Pin = HIGH) {
    delay(5000);
    config_mode_enabled = true;
  }
}

// Incase I want to add an OLED screen for diagnostic / QR code.
// void sleepDisplay(Adafruit_SSD1306* display) {
//   display->ssd1306_command(SSD1306_DISPLAYOFF);
// }

// void wakeDisplay(Adafruit_SSD1306* display) {
//   display->ssd1306_command(SSD1306_DISPLAYON);
// }

//void updateTwitterStatus(String tsData)
//{
//  if (client.connect(thingSpeakAddress, 80))
//  { 
//    // Create HTTP POST Data
//    tsData = "api_key="+thingtweetAPIKey+"&status="+tsData;
//            
//    client.print("POST /apps/thingtweet/1/statuses/update HTTP/1.1\n");
//    client.print("Host: api.thingspeak.com\n");
//    client.print("Connection: close\n");
//    client.print("Content-Type: application/x-www-form-urlencoded\n");
//    client.print("Content-Length: ");
//    client.print(tsData.length());
//    client.print("\n\n");
//
//    client.print(tsData);
//    
//    lastConnectionTime = millis();
//    
//    if (client.connected())
//    {
//      Serial.println("Connecting to ThingSpeak...");
//      Serial.println();
//      
//      failedCounter = 0;
//    }
//    else
//    {
//      failedCounter++;
//  
//      Serial.println("Connection to ThingSpeak failed ("+String(failedCounter, DEC)+")");   
//      Serial.println();
//    }
//    
//  }
//  else
//  {
//    failedCounter++;
//    
//    Serial.println("Connection to ThingSpeak Failed ("+String(failedCounter, DEC)+")");   
//    Serial.println();
//    
//    lastConnectionTime = millis(); 
//  }
//}

void setup() {

  //initializes serial output
  Serial.begin(115200);
  define_pins();

  //debug output displaying the wifi is disabled
  disable_wifi_on_boot();

  //do we enter config_mode?
  //config_mode();

  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  //time to read the moisture value and store it
  moisture_sensor();

  //time to read the light value and store it
  light_sensor();

  //time to read the bme values and store them
  bme_sensor();

  //time to read the battery values and store them
  battery_sensor();

  //now that we have read the sensor values we can upload it
  //to do this, we must turn on the wifi.
  enable_wifi();

  upload_sensor_reading();
  //updateTwitterStatus();
  deep_sleep();
}

void loop() {}
