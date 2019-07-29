//area 1: definitions
//ESP Wifi libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <asyncwebserver.h>

//TZAPU Captive portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

//ThingSpeak library to upload data
#include <ThingSpeak.h>

//sensors
//
//BME280 (Temp,Humidity,Pressure)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
//BH1750 (Lux)
#include <BH1750FVI.h>
//Battery Sensor (Volts,%)
#include <MAX17043.h>

//Screens
#include <U8x8lib.h>

//area 2: Constants

//all nodes()
battery_percent (float)
battery_capacity (float)
wifi_ssid (string)
wifi_password (string)
	
//remote node
unsigned long delayTime;
moisture_level (int)
lux_level (uint16_4)
plant_name (string)
set_mousture_level (int)
//Thingspeak
unsigned long myChannelNumber = ;  // Replace the 0 with your channel number
const char * myWriteAPIKey = "";    // Paste your ThingSpeak Write API Key between the quotes

//central node
temperature (float)
pressure (float)
humidity (float)

//possible to retreive this?
//Or set the monitor 1 meter above ground?
#define SEALEVELPRESSURE_HPA (1021)

Adafruit_BME280 bme; // I2C

// Create the Lightsensor instance
BH1750 lightMeter(0x23);

//Set oled driver
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//define moisture sensor
moisture_power = D3
set pin moisture_power, OUTPUT
set pin moisture_power, LOW
moisture_sensor = A0
set pin moisture_sensor, INPUT

//Define motor pin and set it
Motor = D4
set pin Motor, OUTPUT
set pin Motor, LOW

//define config_button
Config_button = D5
set pin Config_button, INPUT

#wifi is off
#i2c are off (max17943/bh1750/bme280)
#Analogue is off 

read_battery_level{
	wake_sensor
	read_battery_percentage
	read_battery_capacity
	sleep_sensor
	}

check_battery_level{
	is battery level > min_battery_value?
		>yes
			continue
		>no
			change update interval to 4 hours
			send http warning - battery needs charging/replaced
			sleep;

read_soil_moisture();{
	//enable moisture sensor
	set moisture_power HIGH
	read moisture_sensor //moisture_level
	//to stop the plant being over flowed if the sensor stops working
	//so we set a min value to check if the sensor is inserted.
	is moisture_sensor > min_moisture_value?
	>No{set moisture_sensor 1023}
	>Yes{}
	set moisture_power LOW
}

//check whether the soil moisture is below set level
//if it's below the level, turn the mostor on and wait 
//for it to reach the desired level. 
water_plant{
	if moisture_level < desired_moisture_level{
		set Motor, HIGH
    		while moisture_level < desired_moisture_level do {
			read_soil_moisture();
			}
		set Motor, LOW
	}
}

light_sensor(){
Wire.begin();
lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)
float lux_level = lightMeter.readLightLevel();
}

wifi.config(){
//use this web page to add/remove or enable/disable certain sensors	
//i.e. 
//	enable bh1750 {mode} {sensitivity} {i2c address}
//	enable max17403 {i2c address}
//	enable bme280 {i2c address}
// use oled/webpage to show i2c addresses.
//
// use this webpage to enter API keys/URL for content upload

startup mode
	has the config button been pressed?
	>yes
		//disable sleep
		run wifi.setup wizard (ADHOC mode)
		//wifi.name should be the dns.name
		populate the fields from memory.
	>no
		continue
		
Upload_data(){
	wifi.enable;
	send data to MQTT / ThingSpeak
	//battery,lux,moisture,temp,humidity,pressure
	wifi.disable;
}

deep_sleep{
	sleep for x mins
}

>[Start loop]{
//begin.serial;
check_battery_level;
wifi.config;
Read light_sensor;
Read soil_moisture;
water_plant;
wifi.begin;
Upload_data;
Deep_sleep;
}
