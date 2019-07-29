area 1: definitions

area 2: Constants

//all nodes
battery_percent (float)
battery_capacity (float)

//remote node
moisture_level (int)

//remote nodes
lux_level (int)

//central node
temperature (float)
pressure (float)
humidity (float)

set pin D3, LOW

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
	read A0 //moisture_level
}

//check whether the soil moisture is below set level
//if it's below the level, turn the mostor on and wait 
//for it to reach the desired level. 
water_plant{
	if moisture_level < desired_moisture_level{
		set pin D3, HIGH
    while moisture_level < desired_moisture_level do {
			read_soil_moisture();
			}
		set pin D3, LOW
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
