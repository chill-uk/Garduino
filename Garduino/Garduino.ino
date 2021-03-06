// [Open Source Garduino]

#include <ESP8266WiFi.h>
#include <BH1750FVI.h>
#include <ThingSpeak.h>
// Do I need this?
#include <Arduino.h>
// #include <U8x8lib.h>
#include <Wire.h>
#include <MAX17043.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>

#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC

#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "secrets.h"

// [Defining our secrets]
// Wifi config
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWD;
// ThingSpeak
unsigned long myChannelNumber = MY_CHANNEL_NUMBER;  // Replace the 0 with your channel number
const char * myWriteAPIKey = MY_WRITE_API_KEY;    // Paste your ThingSpeak Write API Key between the quotes

// [Create the BME280 instance]
Adafruit_BME280 bme;
// [Create the INA219 instance]
Adafruit_INA219 ina219;

// [Create instances for UDP and NTP]
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// [Create the DS3231 RTC instance]
DS3232RTC extRTC;

// [Create the OLED instance] - Future ideas
// U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// [Create the Lightsensor instance]
BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes);
// BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);   // Alternative mode
// [Create the Thingspeak client instance
WiFiClient client;

// [Define our GPIO pins]
int moistureLevelSensorPin = A0; // Analog input pin for the Moisture Sensor
// int SDL = D1;                 // Used by the wire library for I2C
// int SDA = D2;                 // Used by the wire library for I2C
int moistureLevelPowerPin = D3;  // GPIO pin to power the Moisture Sensor Power
#if defined(LED_BUILTIN)
const int builtInLedPin = LED_BUILTIN;
#else
const int builtInLedPin = D4; // manually configure LED pin
#endif
// int builtInLedPin = D4;          // ESP built in LED (Active Low)
int waterPumpPin = D5;           // GPIO pin to power the Water Pump
int sensorPowerPin = D6;         // GPIO pin to power the Sensors
int lowWaterIndicatorPin = D7;   // Low water level indicator
int configModePin = D8;          // Button to enable booting into diagnostic mode

// [Moisture Sensor variables]
bool waterLevel;
bool capacitiveSensor = false;  // Set to true if you have a capacitive soil moisture sensor
uint16_t startMoistureLevel;
uint16_t soilMoistureLevel;
uint16_t currentMoistureLevel;
uint16_t failsafeMoistureLevel = 3;  // Moisture Level of dry soil
uint16_t minMoistureLevel = 30;      // Moisture Level of dry soil
uint16_t maxMoistureLevel = 70;      // Moisture Level of wet soil
uint16_t maxWateringTime = 10000;    // 10 Seconds

// [Start and End watering times]
// I might want to perform a sunrise/sunset api call to automate this.
uint16_t startWateringTimeHour = 8;
uint16_t endWateringTimeHour = 20;

// [Light Sensor]
uint16_t luxBH1750FVI;

// [Battery Sensor]
float batteryVoltageMAX17043;
float batteryPercentageMAX17043;
float regulatedVoltageMAX17043;

// [INA219 Sensor]
float solarShuntVoltage = 0;
float solarBusVoltage = 0;
float solarCurrent_mA = 0;
float solarLoadVoltage = 0;
float solarPower_mW = 0;

// [BME280 Sensor]
float temperatureBME280;
float pressureBME280;
float altitudeBME280;
float humidityBME280;

// Read this from the internet and store in SPIF//
int SEALEVELPRESSURE_HPA = 1010;

// [Deep sleep time in minutes]
//uint16_t interval = 300;     // 5 Mins
//uint16_t interval = 600;     // 10 Mins
uint16_t interval = 900;       // 15 Mins
//uint16_t interval = 1800;    // 30 Mins
//uint16_t interval = 3600;    // 60 Mins
//uint16_t interval = 30;      // custom time

// [Time structure]
time_t t;

// [RTC settings]
bool RTCSet = false;

// [Deep sleep time with compensation]
unsigned long sleepTime;              // Calculated sleep time variable
uint16_t timeErrorAdjustment = 1070;  // Adjustment in % * 10
// uint16_t timeErrorAdjustment = 1030;  // Sleep clock is slow by +3%
// uint16_t timeErrorAdjustment = 970;  //  Sleep clock is fast by +3%

// [RTC Timezone offser]
uint16_t timeOffset = 3600; //time offset in seconds

// preinit() is called before system startup
// from nonos-sdk's user entry point user_init()

void preinit() {
  // Global WiFi constructors are not called yet
  // (global class instances like WiFi, Serial... are not yet initialized)..
  // No global object methods or C++ exceptions can be called in here!
  //The below is a static class method, which is similar to a function, so it's ok.
  ESP8266WiFiClass::preinitWiFiOff();
}

void definePins() {
  // Set up pin modes:
  pinMode(moistureLevelPowerPin, OUTPUT);
  digitalWrite(moistureLevelPowerPin, LOW);
  pinMode(builtInLedPin, OUTPUT);
  digitalWrite(builtInLedPin, HIGH);
  pinMode(waterPumpPin, OUTPUT);
  digitalWrite(waterPumpPin, LOW);
  pinMode(sensorPowerPin, OUTPUT);
  digitalWrite(sensorPowerPin, LOW);
  pinMode(lowWaterIndicatorPin, INPUT_PULLUP);
  //    digitalWrite(lowWaterIndicatorPin, HIGH);
  pinMode(configModePin, INPUT_PULLUP);
}

void enablePower(int Pin) {
  if (digitalRead(Pin) == LOW) {
    digitalWrite(Pin, HIGH);
  }
}

void disablePower(int Pin) {
  if (digitalRead(Pin) == HIGH) {
    digitalWrite(Pin, LOW);
  }
}

uint16_t readMoistureSensor() {
  // Read sensor value(s)
  uint16_t sensorValue = analogRead(moistureLevelSensorPin);
  if (capacitiveSensor == true) {
    // Map capacitive readings to percentages
    soilMoistureLevel = map(sensorValue, 1400, 3400, 0, 100);
  }
  else {
    // Map conductive reading to percentages
    soilMoistureLevel = map(sensorValue, 0, 1023, 0, 100);
  }
  return soilMoistureLevel;
}

bool readWaterLevel() {
  waterLevel = digitalRead(lowWaterIndicatorPin);
  return waterLevel;
}

void sensorReadBH1750FVI() {
  // Read sensor value(s)
  luxBH1750FVI = LightSensor.GetLightIntensity();
  // Sleep?

  Serial.println("Lux: " + String(luxBH1750FVI));
  Serial.println("----------------------------------");
}

void sensorReadBME280() {
  // Read sensor value(s)
  temperatureBME280 = bme.readTemperature();
  pressureBME280 = bme.readPressure() / 100.0F;
  altitudeBME280 = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidityBME280 = bme.readHumidity();
  // Sleep?

  Serial.println("Air Temperature = " + String(temperatureBME280) + " *C");
  Serial.println("Air Pressure = " + String(pressureBME280) + " hPa");
  Serial.println("Approx. Altitude = " + String(altitudeBME280) + " m");
  Serial.println("Air Humidity = " + String(humidityBME280) + " %");
  Serial.println("----------------------------------");
}

void sensorReadMAX17043() {
  // Read sensor value(s)
  delay(500);
  batteryPercentageMAX17043 = (FuelGauge.percent());
  regulatedVoltageMAX17043 = (FuelGauge.adc());
  batteryVoltageMAX17043 = ((FuelGauge.voltage()) / 1000);
  regulatedVoltageMAX17043 = (regulatedVoltageMAX17043 / 1000);
  FuelGauge.sleep();

  Serial.println("Battery Voltage = " + String(batteryVoltageMAX17043) + "V");
  Serial.println("Battery Percentage = " + String(batteryPercentageMAX17043) + "%");
  Serial.println("Regulated Voltage = " + String(regulatedVoltageMAX17043) + "V");
  Serial.println("----------------------------------");
}

void sensorReadINA219() {
  // Read sensor value(s)
  solarShuntVoltage = ina219.getShuntVoltage_mV();
  solarBusVoltage = ina219.getBusVoltage_V();
  solarCurrent_mA = ina219.getCurrent_mA();
  solarPower_mW = ina219.getPower_mW();
  solarLoadVoltage = solarBusVoltage + (solarShuntVoltage / 1000);

  Serial.print("Bus Voltage:   "); Serial.print(solarBusVoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(solarShuntVoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(solarLoadVoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(solarCurrent_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(solarPower_mW); Serial.println(" mW");
  Serial.println("----------------------------------");
}

void waterPlant() {
  int currentWateringTime = 0;
  int debugTimeMeasurement = millis();
  startMoistureLevel = readMoistureSensor();
  Serial.print("Waterlevel: "); Serial.println(waterLevel);

  if (soilMoistureLevel < failsafeMoistureLevel) {
    Serial.println("Moisture Sensor broken or disconnected");
  }

  else if ((failsafeMoistureLevel <= soilMoistureLevel) && (soilMoistureLevel <= minMoistureLevel)) {
    Serial.println("Start of watering");
    // uint16_t currentMoistureLevel = readMoistureSensor();
    int startWateringTime = millis();
    while ((readMoistureSensor() <= maxMoistureLevel) && (currentWateringTime <= maxWateringTime) && (readWaterLevel() == HIGH)) {
      enablePower(waterPumpPin);
      currentWateringTime = millis() - startWateringTime;
      delay(250); // I tried using yeild(); but I kept getting random LOW values on readWaterLevel(), which exited the loop.
    }
    disablePower(waterPumpPin);
    Serial.print("Moisture Level after watering: "); Serial.println(soilMoistureLevel);
    Serial.print("Moisture Level change: "); Serial.println(soilMoistureLevel - startMoistureLevel);
    Serial.println("Watering time: " + String(currentWateringTime));
  }

  else if ((minMoistureLevel < startMoistureLevel) && (startMoistureLevel <= (maxMoistureLevel + 3))) {
    Serial.println("Plant is happy");
  }
  else if (startMoistureLevel > (maxMoistureLevel + 3)) {
    Serial.println("Plant is too wet");
  }
  if (waterLevel == LOW) {
    Serial.println();
    Serial.println("Water tank is low");
  }
  Serial.println("----------------------------------");
}

void checkBatteryLevel() {
  sensorReadMAX17043();
  // Sort out this case statement

  // case(batteryVoltageMAX17043) {
  //     <= 3.8 {
  //         Serial.println("Battery is good");
  //         // wifi and watering pump enabled
  //         }
  //     <= 3.6  && < 3.8 {
  //         Serial.println("Battery is Medium");
  //         // wifi and watering pump enabled
  //         // "Battery Medium" warning message sent
  //         }
  //     > 3.6 {
  //         Serial.println("Battery is low");
  //         // wifi enabled every 2 hours
  //         // watering pump disabled
  //         // "Battery Low" warning message sent
  //         // Turn on Red LED
  //         }
  // }
}

void connectWifi()
{
  int maxWifiConnectionTime = 10;
  int countDownTimer = 0;

#if defined(ESP8266) || defined(ESP32)
  WiFi.mode(WIFI_STA);
#endif

  Serial.println("Connecting to: " + String(ssid));
  // Serial.println(ssid);

  WiFi.begin(ssid, pass);

  bool blink = true;
  int currentWifiConnectionTime = 0;
  int startWifiConnectionTime = millis();
  Serial.print("Connection time left = ");
  while ((WiFi.status() != WL_CONNECTED) && (currentWifiConnectionTime <= (maxWifiConnectionTime * 1000))) {
    //        WiFi.status();
    currentWifiConnectionTime = (millis() - startWifiConnectionTime);
    // try and print seconds into Serial output
    int countDownSeconds = (currentWifiConnectionTime / 1000);
    if (countDownSeconds == countDownTimer) {
      Serial.print(maxWifiConnectionTime - countDownTimer);
      Serial.print("..");
      digitalWrite(builtInLedPin, blink ? LOW : HIGH); // active low led
      blink = !blink;
      countDownTimer++;
    }
    yield();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("WiFi connected in "); Serial.print(currentWifiConnectionTime); Serial.println("Seconds");
  }
  else {
    Serial.println("");
    Serial.print("Couldn't connect to wifi within the allocated time ("); Serial.print(maxWifiConnectionTime); Serial.println(" Seconds)");
    Serial.println("Please adjust the maxWifiConnectionTime or imporve singal strength");
  }
  digitalWrite(builtInLedPin, HIGH); // active low led
  Serial.println("----------------------------------");
}

void disconnectWifi() {
  WiFi.disconnect(true);
  delay(1);
  Serial.print("Wifi disconnected: "); Serial.println(WiFi.disconnect() ? "YES" : "NO");
  Serial.println("----------------------------------");
}

void enterDeepSleep() {
  disconnectWifi();
  if (RTCSet = true) {
    // This calculates the amount of time needed to sleep to wake up
    sleepTime = ((interval - (extRTC.get() % interval)) * (timeErrorAdjustment)); // Sleep time in milliseconds (mS)
    Serial.print("Sleep time: "); Serial.print(sleepTime / 1000); Serial.println(" seconds");
    // use [WAKE_RF_DISABLED] to keep the WiFi radio disabled when it wakes up
    ESP.deepSleep(((sleepTime) * 1000), WAKE_RF_DEFAULT); // sleep time in microseconds (uS)
  }
  else {
    Serial.print("Sleep time: 15 minutes");
    // use [WAKE_RF_DISABLED] to keep the WiFi radio disabled when it wakes up
    ESP.deepSleep((15e6), WAKE_RF_DEFAULT); // sleep time in microseconds (uS)
  }
  Serial.println("----------------------------------");
}

void uploadSensorReading()
{
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  ThingSpeak.begin(client);
  ThingSpeak.setField(1, luxBH1750FVI);
  ThingSpeak.setField(2, temperatureBME280);
  ThingSpeak.setField(3, humidityBME280);
  ThingSpeak.setField(4, soilMoistureLevel);
  ThingSpeak.setField(5, batteryVoltageMAX17043);
  ThingSpeak.setField(6, batteryPercentageMAX17043);
  ThingSpeak.setField(7, solarLoadVoltage);
  ThingSpeak.setField(8, solarCurrent_mA);


  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  // Check the return code
  if (x == 200) {
    Serial.println("Channel update successful.");
  }
  else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  Serial.println("----------------------------------");
}

void setExternalRTC() {

  Serial.println();
  Serial.println("Setting RTC from NTP server");
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  timeClient.begin();
  timeClient.setTimeOffset(timeOffset);
  timeClient.update();
  delay(1); //not sure if this is needed
  extRTC.set(timeClient.getEpochTime());

  setSyncProvider(extRTC.get);   // the function to get the time from the RTC
  Serial.print("extRTC post-epoch"); Serial.println(extRTC.get());
  Serial.print("Internal post-epoch"); Serial.println(timeClient.getEpochTime());
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  }
  else {
    Serial.println("RTC has set the system time");
    RTCSet = true;
  }
  Serial.println("----------------------------------");
}

void readExtRTC() {
  char buf[40];
  t = extRTC.get();

  Serial.print("extRTC pre-epoch"); Serial.println(t);
  sprintf(buf, "%.2d:%.2d:%.2d %.2d%s%d ",
          hour(t), minute(t), second(t), day(t), monthShortStr(month(t)), year(t));
  Serial.println(buf);
  // Serial.print("extRTC.ocsStopped: ");  Serial.println(extRTC.oscStopped());

  // set extRTC if stopped -- Maybe check if year is 1970?
  // if ((extRTC.oscStopped() == 1) || ((day(t) == 01) && (hour(t) == 00) && (minute(t) == 00)) || (t <= 1000)) {
  if ((extRTC.oscStopped() == 1) || (year(t) == 1970) || (t <= 1000)) {
    setExternalRTC();
  }
  else {
    Serial.println("extRTC seems fine");
    setSyncProvider(extRTC.get);   // the function to get the time from the RTC
  }
  Serial.println("----------------------------------");
}

void configMode() {
  Serial.println("Config mode entered");


  // do some config stuff?
  // Maybe turn the oled screen on
  // turn on the web server
  // Allow you to change the settings?
}

void enablePeripherals() {
  Serial.begin(9600);
  extRTC.begin();
  FuelGauge.begin();
  FuelGauge.wake();
  ina219.begin();
  ina219.setCalibration_16V_400mA();
  bme.begin(0x76);
  LightSensor.begin();
  Serial.println("");
}

void setup() {

  definePins();
  enablePeripherals();

  // [configMode]
  if (configModePin == HIGH) {
    configMode();
  }
  Serial.println(readWaterLevel());
  readExtRTC();

  checkBatteryLevel();
  sensorReadBH1750FVI();
  bme.takeForcedMeasurement(); // has no effect in normal mode
  sensorReadBME280();
  sensorReadINA219();

  enablePower(moistureLevelPowerPin);

  Serial.print("Soil Moisture Level: "); Serial.println(readMoistureSensor());
  Serial.println("----------------------------------");

  if ((startWateringTimeHour <= (hour(t))) && ((hour(t)) < endWateringTimeHour)) {
    waterPlant();
  }
  else {
    Serial.println("Not watering - Outside of watering hours.");
    Serial.println("----------------------------------");
  }
  disablePower(moistureLevelPowerPin);

  uploadSensorReading();
  // updateTwitterStatus();
  enterDeepSleep();
}

void loop() {
  // Don't put anything here
}
