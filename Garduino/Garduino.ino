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
int builtInLedPin = D4;          // ESP built in LED (Active Low)
int waterPumpPin = D5;           // GPIO pin to power the Water Pump
int sensorPowerPin = D6;         // GPIO pin to power the Sensors
int lowWaterIndicatorPin = D7;   // Low water level indicator
int configModePin = D8;          // Button to enable booting into diagnostic mode

// [Moisture Sensor variables]
uint16_t soilMoistureLevel;
uint16_t failsafeMoistureLevel = 20;  // Moisture Level of dry soil
uint16_t minMoistureLevel = 300;      // Moisture Level of dry soil
uint16_t maxMoistureLevel = 700;      // Moisture Level of wet soil
uint16_t maxWateringTime = 10000;     // 10 Seconds

// [Light Sensor]
uint16_t luxBH1750FVI;
//unsigned long delayTime;  <--??

// [Battery Sensor]
float batteryVoltageMAX17043;
float batteryPercentageMAX17043;
uint16_t regulatedVoltageMAX17043;

// [BME280 Sensor]
float temperatureBME280;
float pressureBME280;
float altitudeBME280;
float humidityBME280;
// Read this from the internet and store in SPIF//
int SEALEVELPRESSURE_HPA = 1010;

// [Deep sleep time]
// int sleepIntervalTime = 5;     // 5 Mins
// int sleepIntervalTime = 10;     // 10 Mins
int sleepIntervalTime = 15;        // 15 Mins
// int sleepIntervalTime = 30;    // 30 Mins
// int sleepIntervalTime = 60;    // 60 Mins

// [RTC Timezone offser]
int timeOffset = 3600; //time offset in seconds

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

void sensorReadBH1750FVI() {
    // Wake?
    luxBH1750FVI = LightSensor.GetLightIntensity();
    // Sleep?
    Serial.print("Lux: "); Serial.println(luxBH1750FVI);
}

void sensorReadBME280() {
    // Wake?
    temperatureBME280 = bme.readTemperature();
    pressureBME280 = bme.readPressure() / 100.0F;
    altitudeBME280 = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidityBME280 = bme.readHumidity();
    // Sleep?
    Serial.print("Air Temperature = ");     Serial.print(temperatureBME280); Serial.println(" *C");
    Serial.print("Air Pressure = ");        Serial.print(pressureBME280);    Serial.println(" hPa");
    Serial.print("Approx. Altitude = ");    Serial.print(altitudeBME280);    Serial.println(" m");
    Serial.print("Air Humidity = ");        Serial.print(humidityBME280);    Serial.println(" %");
}

void sensorReadMAX17043() {
    FuelGauge.wake();
    batteryPercentageMAX17043 = FuelGauge.percent();
    regulatedVoltageMAX17043 = FuelGauge.adc();
    batteryVoltageMAX17043 = FuelGauge.voltage();
    FuelGauge.sleep();
    Serial.print("Battery Voltage = ");     Serial.print(batteryVoltageMAX17043);   Serial.println("V");
    Serial.print("Battery Percentage = ");  Serial.print(batteryPercentageMAX17043);Serial.println("%");
    Serial.print("Regulated Voltage = ");   Serial.print(altitudeBME280);           Serial.println("V");
}

void waterPlant() {
    int currentMoistureLevel = 0;
    int currentWateringTime = 0;
    
    enablePower(moistureLevelPowerPin);
    int firstMoistureLevel = analogRead(moistureLevelSensorPin);
    //moisture_sensor_sleep();
    Serial.print("First Moisture Level: "); Serial.println(firstMoistureLevel);
    
    int timeMeasurement = millis();
    
    if ((firstMoistureLevel > failsafeMoistureLevel) && (firstMoistureLevel <= minMoistureLevel)) {
        Serial.println("Start of watering");
        int startWateringTime = millis();
        while ((currentMoistureLevel <= maxMoistureLevel) && (currentWateringTime <= maxWateringTime)) {
            currentWateringTime = millis() - startWateringTime;
            currentMoistureLevel = analogRead(moistureLevelSensorPin);
            // Serial.print("Current Moisture Level: "); Serial.println(currentMoistureLevel);
            // Serial.print("Max Moisture Level: "); Serial.println(maxMoistureLevel);
            enablePower(waterPumpPin);
            yield();
        // delay(100);
        }
    }
    disablePower(moistureLevelPowerPin);
    disablePower(waterPumpPin);

// clean the following code up a bit
    if (firstMoistureLevel <= failsafeMoistureLevel) {
        Serial.println("Moisture Sensor broken or disconnected");
    }
    if ((firstMoistureLevel <= minMoistureLevel) && (firstMoistureLevel > failsafeMoistureLevel)) {
        Serial.println("Plant needed watering");
        Serial.print("Moisture Level change: "); Serial.println(currentMoistureLevel - firstMoistureLevel);  
        Serial.print("Watering time: "); Serial.println(currentWateringTime);
    }
    if ((firstMoistureLevel > minMoistureLevel) && (firstMoistureLevel <= (maxMoistureLevel+50))) {
        Serial.println("Plant is happy");
    }
    if (firstMoistureLevel > (maxMoistureLevel+50)) {
        Serial.println("Plant is too wet");
    }

    Serial.print("Execute time: "); Serial.println(millis() - timeMeasurement);
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
    WiFi.mode(WIFI_STA);
    Serial.print("Connecting to: ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass); 
    while (WiFi.status() != WL_CONNECTED) 
    {
    //try not using delay
    delay(500);
    Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected"); 
}

void disconnectWifi() {
  WiFi.disconnect(true);
  delay(1);
  Serial.println(WiFi.disconnect());
}

void enterDeepSleep() {
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when it wakes up
  //  int getDay() const;
  //  int getHours() const;

    int Minutes_Left = (sleepIntervalTime-(timeClient.getMinutes()%(sleepIntervalTime))-1);
  //  int Seconds_Left = (60-(timeClient.getSeconds()));
    int Sleep_Time = ((Minutes_Left*60)+(60-(timeClient.getSeconds())));
  // int Sleep_Time = Wake_Up_Interval-(extRTC.get%Wake_Up_Interval);
  Serial.print("Going into deep sleep mode for "); Serial.print(Sleep_Time); Serial.print(" seconds");
  ESP.deepSleep((Sleep_Time*1000), WAKE_RF_DISABLED); // remember to fix this to enable wifi
}

void uploadSensorReading()
{
    ThingSpeak.begin(client); 
    ThingSpeak.setField(1, luxBH1750FVI);
    ThingSpeak.setField(2, temperatureBME280);
    ThingSpeak.setField(3, pressureBME280);
    ThingSpeak.setField(4, humidityBME280);
    ThingSpeak.setField(5, soilMoistureLevel);
    ThingSpeak.setField(6, batteryVoltageMAX17043);
    ThingSpeak.setField(7, batteryPercentageMAX17043);

    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    // Check the return code
    if(x == 200){
    Serial.println("Channel update successful.");
    }
    else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
}

void setExternalRTC() {
    connectWifi();
    extRTC.begin();
    timeClient.begin();
    timeClient.setTimeOffset(timeOffset);
    timeClient.update();
    delay(1); //not sure if this is needed
    extRTC.set(timeClient.getEpochTime());

    setSyncProvider(extRTC.get);   // the function to get the time from the RTC
    Serial.println(timeStatus());
    if(timeStatus() != timeSet){
        Serial.println("Unable to sync with the RTC");
    }
    else {
        Serial.println("RTC has set the system time");
    }
}

void initialiseSensors() {
    if (!FuelGauge.begin()) {
        Serial.println(F("Could not find a valid MAX17043 sensor, check wiring!"));
    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    if (!LightSensor.begin()) {
        Serial.println(F("Could not find a valid BH1750FVI sensor, check wiring!"));

}

void configMode() {
    Serial.println("Config mode entered");
    

    // do some config stuff?
    // Maybe turn the oled screen on
    // turn on the web server
    // Allow you to change the settings?
}

void setup() {
    
    definePins();
    Serial.begin(9600);
    enablePower(sensorPowerPin);

    // configMode
    if (configModePin == HIGH) {
        configMode();
    }
    else {
    initialiseSensors();
    }

    if (extRTC.oscStopped() == 1) { 
        setExternalRTC();
    }
    checkBatteryLevel();

    sensorReadBH1750FVI();
    //bme.takeForcedMeasurement(); // has no effect in normal mode
    sensorReadBME280();
    disablePower(sensorPowerPin);

    // if ((firstMoistureLevel > failsafeMoistureLevel) {
    //    && (RTC(Mins) == 0) && )
    waterPlant();

    uploadSensorReading();
    //updateTwitterStatus();
    enterDeepSleep();
}

void loop() {
    // Don't put anything here
}
