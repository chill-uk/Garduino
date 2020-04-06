// Garduino

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
// change next line to use with another board/shield
#include <ESP8266WiFi.h>
//#include <WiFi.h> // for WiFi shield
//#include <WiFi101.h> // for WiFi 101 shield or MKR1000
#include <WiFiUdp.h>

// Read this from the internet and store in SPIF//
int SEALEVELPRESSURE_HPA = 1010;

Adafruit_BME280 bme; // I2C
WiFiUDP ntpUDP;
DS3232RTC extRTC;
NTPClient timeClient(ntpUDP);

// Create the Lightsensor instance
BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes);
// BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);


const char *ssid =  "";
const char *pass =  "";

#ifndef STASSID
#define STASSID ""
#define STAPSK  ""
#endif

unsigned long myChannelNumber = ;  // Replace the 0 with your channel number
const char * myWriteAPIKey = "";    // Paste your ThingSpeak Write API Key between the quotes

int Moisture_Level_Sensor = A0; // select the input pin for the Moisture Sensor
int Moisture_Level_Power = D3;  // select the pin for the Moisture Sensor Power
int Motor_Power_Pin = D5;       // select the pin for the Water_Motor
int Sleep_Led = D4;             // Display sleep mode

uint16_t moisture_level;
uint16_t Failsafe_Moisture_Level = 20; // Moisture Level of dry soil
uint16_t Min_Moisture_Level = 300; // Moisture Level of dry soil
uint16_t Max_Moisture_Level = 700; // Moisture Level of wet soil
uint16_t Max_Watering_Time = 10000; // 10 Seconds
uint16_t lux;
unsigned long delayTime;

float = battery_voltage;
float = battery_percent;
uint16_t = regulated_voltage;

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

void define_pins() {
    // Set up pin modes:
    pinMode(Moisture_Level_Power, OUTPUT);
    digitalWrite(Moisture_Level_Power, LOW);  
    pinMode(Motor_Power_Pin, OUTPUT);
    digitalWrite(Motor_Power_Pin, LOW);  
    pinMode(Sleep_Led, OUTPUT);
    digitalWrite(Motor_Power_Pin, HIGH);  
}

void moisture_sensor_read() {
    if (digitalRead(Moisture_Level_Power) == LOW) {
        digitalWrite(Moisture_Level_Power, HIGH); // Turn on moisture sensor to make a reading
    }
    return analogRead(Moisture_Level_Sensor);
}

void moisture_sensor_sleep() {
    if (digitalRead(Moisture_Level_Power) == HIGH) {
        digitalWrite(Moisture_Level_Power, LOW); // Turn on moisture sensor to make a reading
    }
}

void light_sensor_read() {
    LightSensor.wake();
    lux = LightSensor.GetLightIntensity();
    LightSensor.sleep();
    Serial.print("Light: "); Serial.println(lux);
}

void bme280_sensor_read() {
    bme.wake();
    external_temperature = bme.readTemperature();
    external_pressure = bme.readPressure() / 100.0F;
    external_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    external_humidity = bme.readHumidity();
    bme.sleep();
    Serial.print("Temperature = ");      Serial.print(external_temperature); Serial.println(" *C");
    Serial.print("Pressure = ");         Serial.print(external_pressure);    Serial.println(" hPa");
    Serial.print("Approx. Altitude = "); Serial.print(external_altitude);    Serial.println(" m");
    Serial.print("Humidity = ");         Serial.print(external_humidity);    Serial.println(" %");
}

void enable_water_pump();
    if (digitalRead(Motor_Power_Pin) == LOW) {
        digitalWrite(Motor_Power_Pin, HIGH);
    }
}

void disable_water_pump();
    if (digitalRead(Motor_Power_Pin) == HIGH) {
        digitalWrite(Motor_Power_Pin, LOW);
    }
}

void water_plant() {
    int Time_Measurement = 0;
    int Current_Moisture_Level = 0;
    int Start_Watering_Time = 0;
    int Current_Watering_Time = 0;

    Serial.println("Start of watering");

    Time_Measurement = millis();

    if ((First_Moisture_Level <= Min_Moisture_Level) && (First_Moisture_Level > Failsafe_Moisture_Level)) {
        Start_Watering_Time = millis();
        while ((Current_Moisture_Level <= Max_Moisture_Level) && (Current_Watering_Time <= Max_Watering_Time)) {
            Current_Watering_Time = millis() - Start_Watering_Time;
            Current_Moisture_Level = moisture_sensor_read();
            // Serial.print("Current Moisture Level: "); Serial.println(Current_Moisture_Level);
            // Serial.print("Max Moisture Level: "); Serial.println(Max_Moisture_Level);
            enable_water_pump();
        //? yelid();
        delay(100);
        }
    }
    moisture_sensor_sleep();
    disable_water_pump();

// clean the following code up a bit
    if (First_Moisture_Level <= Failsafe_Moisture_Level) {
        Serial.println("Moisture Sensor broken or disconnected");
    }
    if ((First_Moisture_Level <= Min_Moisture_Level) && (First_Moisture_Level > Failsafe_Moisture_Level)) {
        Serial.println("Plant needed watering");
        Serial.print("Moisture Level change: "); Serial.println(Current_Moisture_Level - First_Moisture_Level);  
        Serial.print("Watering time: "); Serial.println(Current_Watering_Time);
    }
    if ((First_Moisture_Level > Min_Moisture_Level) && (First_Moisture_Level <= (Max_Moisture_Level+50))) {
        Serial.println("Plant is happy");
    }
    if (First_Moisture_Level > (Max_Moisture_Level+50)) {
        Serial.println("Plant is too wet");
    }

    Serial.print("Execute time: "); Serial.println(millis() - Time_Measurement);
}

void check_battery_level() {
    FuelGauge.wake();
    battery_percent = FuelGauge.percent();
    regulated_voltage = FuelGauge.adc();
    battery_voltage = FuelGauge.voltage();
    
    // Sort out this case statement

    // case(battery_voltage) {
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
    FuelGauge.sleep();
    
    // FuelGauge value types

    // uint16_t adc();
    // float voltage();
    // float percent();
    // bool sleep();
    // bool isSleeping();
    // bool wake();
    // void reset();
    // void quickstart();
    // bool alertIsActive();
    // void clearAlert();
    // uint8_t getThreshold();
    // void setThreshold(uint8_t threshold);
}

void connect_wifi()
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

void disconnect_wifi() {
  WiFi.disconnect(true);
  delay(1);
  Serial.println(WiFi.disconnect());
}

void deep_sleep() {
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when it wakes up
  Serial.print("Going into deep sleep mode for "); Serial.print(SLEEP_TIME/1000); Serial.print(" seconds");
  ESP.deepSleep(SLEEP_TIME, WAKE_RF_DISABLED); // remember to fix this to enable wifi
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

void set_external_RTC() {
    connect_wifi();
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

void initialise_sensors() {
    Serial.begin(9600);
    FuelGauge.begin();
    LightSensor.begin();
    bme.begin();

}

// void Config_Mode() {
//     do some config stuff?
//     Maybe turn the oled screen on
//     turn on the web server
//     Allow you to change the settings?
// }

void setup() {
    define_pins();
    initialise_sensors();
    // check_wake_reason
    if (ESP.getResetReason() = "Power On") {
        set_external_RTC();
    }
    check_battery_level();

    //time to read the light value and store it
    light_sensor_read();
    //bme.takeForcedMeasurement(); // has no effect in normal mode
    //time to read the bme values and store them
    bme_sensor_read();

    //time to read the moisture value and store it
    First_Moisture_Level = moisture_sensor_read();
    moisture_sensor_sleep();
    Serial.print("First Moisture Level: "); Serial.println(First_Moisture_Level);

    // Config_Mode
    // if (config_pin_x is HIGH) {
    //  Config_Mode();
    // }

    // if ((First_Moisture_Level > Failsafe_Moisture_Level) {
    //    && (RTC(Mins) == 0) && )
    water_plant();

    upload_sensor_reading();
    //updateTwitterStatus();
    deep_sleep();
}

void loop() {}