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
			read A0 //moisture_level
			}
		set pin D3, LOW
	}
}

light_sensor(){
bh1750.begin(one_time_value)
}

begin.wifi(){
has wifi already been setup?
if not, run the special set up wizard.

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
reset_mode;
wifi.begin;
Read light_sensor
Read soil_moisture
water_plant;
Upload_data;
Deep_sleep;
}
