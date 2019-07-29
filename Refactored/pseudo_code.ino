const char *ssid =  "";
const char *pass =  "";
int moisture_level;
uint16_t lux;
unsigned long myChannelNumber = ;  // Replace the 0 with your channel number
const char * myWriteAPIKey = "";    // Paste your ThingSpeak Write API Key between the quotes
unsigned long delayTime;

area 1: definitions

#include <ESP8266WiFi.h>
#include <BH1750FVI.h>
#include <ThingSpeak.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <MAX17043.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <asyncwebserver.h>

area 2: Constants

//all nodes()
battery_percent (float)
battery_capacity (float)
wifi_ssid (string)
wifi_password (string)
	
//remote node
moisture_level (int)
lux_level (int)
plant_name (string)
set_mousture_level (int)
Thingspeak_API_key (string)

	
//central node
temperature (float)
pressure (float)
humidity (float)

//possible to retreive this?
//Or set the monitor 1 meter above ground?
#define SEALEVELPRESSURE_HPA (1021)

Adafruit_BME280 bme; // I2C
// Create the Lightsensor instance
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

//Set oled driver
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//define moisture sensor
moisture_power = D3
set pin moisture_power, OUTPUT
set pin moisture_power, LOW
moisture_sensor = A0
set pin moisture_sensor, INPUT

//define config_button
Config_button = D5
set pin Config_button, INPUT

//Define motor pin and set it
Motor = D4
set pin Motor, OUTPUT
set pin Motor, LOW

#wifi if off
#i2c are off (max17943/bh1750/bme280)
#Analogue is off 

reset_mode(){
	find out how to turn on the MCU based on a button press
	if button pressed for 3 seconds {
		beep
		reset wifi settings;
	}
}		

read_soil_moisture();{
	//enable moisture sensor
	set moisture_power HIGH
	read moisture_sensor //moisture_level
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
bh1750.begin(one_time_value)
}

wifi.begin(){
//has wifi already been setup?
//if not, run the special set up wizard.
//use this web page to add/remove or enable/disable certain sensors	
//i.e. 
//	enable bh1750 {mode} {sensitivity} {i2c address}
//	enable max17403 {i2c address}
//	enable bme280 {i2c address}
// use oled/webpage to show i2c addresses.
//
// use this webpage to enter API keys/URL for content upload

startup mode
	is there a crc checksum for the wifi?
	>yes
		has the config button been pressed?
		>yes
			disable sleep
			run wifi.setup wizard (ADHOC mode) 
			populate the fields from memory.
		>no
			load the wifi settings 
	>no
		disable sleep
		run the wifi.setup wizard (ADHOC mode)
restart the MCU 
		
Upload_data(){
	wifi.enable;
	send data to MQTT / ThingSpeak
	wifi.disable;
}

deep_sleep{
	sleep for x mins
]

>[Start loop]{
begin.serial;
wifi.begin;
Read light_sensor
Read soil_moisture
water_plant;
Upload_data;
Deep_sleep;
}
