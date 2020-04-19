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

// [Create the DS3231 RTC instance]
DS3232RTC myRTC;
// [Create instances for UDP and NTP]
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

unsigned long sleepTime;              // Calculated sleep time variable
uint16_t interval = 900;              // Sleep time in seconds
uint16_t timeErrorAdjustment = 1030;  // Adjustment in % (multiplied by 1000) // i.e. Sleep clock is fast by +3%
uint16_t timeOffset = 3600;
int sensorPowerPin = D6;              // GPIO pin to power the Sensors


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

void setup()
{
    pinMode(sensorPowerPin, OUTPUT);
    digitalWrite(sensorPowerPin, LOW);
    Serial.begin(9600);
    enablePower(sensorPowerPin);
    myRTC.begin();
    FuelGauge.begin();
    // delay(200);
}

void enterDeepSleep() {
    disablePower(sensorPowerPin);  
    ESP.deepSleep(((sleepTime)*1000), WAKE_NO_RFCAL); // sleep time in microseconds (uS)
}

void loop()
{
    display();
//    delay(Sleep_Time);
    enterDeepSleep();
}

void connectWifi()
{
    int maxWifiConnectionTime = 20;
    int countDownTimer = 0;

    WiFi.mode(WIFI_STA);
    Serial.println("Connecting to: " + String(ssid));
    // Serial.println(ssid);

    WiFi.begin(ssid, pass);
    int currentWifiConnectionTime = 0;
    int startWifiConnectionTime = millis();
    Serial.print("Connection time left = ");
    while ((WiFi.status() != WL_CONNECTED) && (currentWifiConnectionTime <= (maxWifiConnectionTime*1000))) {
        WiFi.status();
        currentWifiConnectionTime = (millis() - startWifiConnectionTime);
        yield();
        // try and print seconds into Serial output
        int countDownSeconds = (currentWifiConnectionTime/1000);
        if (countDownSeconds == countDownTimer) {
            Serial.print(maxWifiConnectionTime - countDownTimer);
            Serial.print("..");
            countDownTimer++;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.print("WiFi connected in "); Serial.print(currentWifiConnectionTime);    Serial.println(" Seconds");
    }
    else {
        Serial.println("");
        Serial.print("Couldn't connect to wifi within the allocated time ("); Serial.print(maxWifiConnectionTime);    Serial.println("Seconds)");
        Serial.println("Please adjust the maxWifiConnectionTime or imporve singal strength");
        enterDeepSleep(); 
    }
}

void setExternalRTC() {

    Serial.println();
    Serial.println("Setting RTC from NTP server");
    connectWifi();
    
    timeClient.begin();
    timeClient.setTimeOffset(timeOffset);
    timeClient.update();
    delay(1); //not sure if this is needed
    myRTC.set(timeClient.getEpochTime());

    setSyncProvider(myRTC.get);   // the function to get the time from the RTC
    Serial.println(timeStatus());
    if(timeStatus() != timeSet){
        Serial.println("Unable to sync with the RTC");
    }
    else {
        Serial.println("RTC has set the system time");
    }
}

void sensorReadMAX17043() {
//    delay(200); //needed to wait until the sensor wakes up
    FuelGauge.wake();
    float batteryPercentageMAX17043 = (FuelGauge.percent());
    float regulatedVoltageMAX17043 = (FuelGauge.adc());
    float batteryVoltageMAX17043 = ((FuelGauge.voltage())/1000);
    regulatedVoltageMAX17043 = (regulatedVoltageMAX17043/1000);
    FuelGauge.sleep();
    Serial.println("Battery Voltage = " + String(batteryVoltageMAX17043) + "V");
    Serial.println("Battery Percentage = " + String(batteryPercentageMAX17043) + "%");
    Serial.println("Regulated Voltage = " + String(regulatedVoltageMAX17043) + "V");
}

// display time, date, temperature
void display()
{
    time_t t = myRTC.get();
    Serial.println(t);
    Serial.print("myRTC.ocsStopped: ");  Serial.println(myRTC.oscStopped());
    // set extRTC if stopped
    if ((year(t) == 1970) || (minute(t) == 00)) { 
        setExternalRTC();
    }
    else {
    Serial.println("extRTC seems fine");      
    }
    char buf[40];
    t = myRTC.get();    
    Serial.println(myRTC.get());    
    Serial.println(t);    
    float celsius = myRTC.temperature() / 4.0;
//    float fahrenheit = celsius * 9.0 / 5.0 + 32.0;
    sprintf(buf, "%.2d:%.2d:%.2d %.2d%s%d ",
        hour(t), minute(t), second(t), day(t), monthShortStr(month(t)), year(t));
//    Serial.println(minute(t));    
//    Serial.println(second(t));
    Serial.println(buf);
    Serial.print("extRTC temp: ");
    Serial.print(celsius);
    Serial.println("C");
//    Serial.print(fahrenheit);
//    Serial.println("F");
    sensorReadMAX17043();
    Serial.print("Sleep time: ");
    sleepTime = ((interval-(myRTC.get()%interval))*timeErrorAdjustment);  // Sleep time in milliseconds (mS)
    Serial.print(sleepTime/1000); // sleep time in seconds (S)
    Serial.println(" seconds");
}
