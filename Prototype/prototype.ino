#include <ESP8266WiFi.h>
#include <BH1750FVI.h>
#include <ThingSpeak.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <MAX17043.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1021)

Adafruit_BME280 bme; // I2C

// Create the Lightsensor instance
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

const char *ssid =  "";
const char *pass =  "";
int moisture_level;
uint16_t lux;
unsigned long myChannelNumber = ;  // Replace the 0 with your channel number
const char * myWriteAPIKey = "";    // Paste your ThingSpeak Write API Key between the quotes
unsigned long delayTime;

WiFiClient client;

void bme280_setup(){
      bool status;
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    Serial.println("Valid BME280 sensor found");
}

void setup() 
{
  Serial.begin(115200);
  bme280_setup();
  //FuelGauge.begin();
  u8x8.begin();
  u8x8.clearDisplay( );
  
  //print_serial();
  //read_light();
  //read_moisture();
  //print_screen();
  
  //connect_to_wifi();
  //upload_sensor_reading();
  //go_to_sleep();
}

void read_bme280(){
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
}

void print_serial()
{
  Serial.println();
  Serial.println("I'm awake");
}

void read_moisture()
{
  moisture_level = analogRead(A0);
  Serial.print("Moisture = ");
  Serial.println (moisture_level);
}

void go_to_sleep()
{
  Serial.println("I'm going into deep sleep mode for 15 seconds");
  ESP.deepSleep(15e6);
  //Serial.println("I'm going into deep sleep mode for 30 mins");
  //ESP.deepSleep(1800000000);
}

void read_light()
{
  LightSensor.begin();
  lux = LightSensor.GetLightIntensity();
  //Serial.println();
  Serial.print("Light: ");
  Serial.println(lux);
}

void connect_to_wifi()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to: ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, pass); 
  while (WiFi.status() != WL_CONNECTED) 
    {
    delay(500);
    Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connected"); 
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

void print_screen()
{
  //u8x8.clearDisplay( );
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  //u8x8.print(VariableInUse); 
  u8x8.print("Temp.= ");
  u8x8.print(bme.readTemperature());
  u8x8.println("*C");

  u8x8.print("Pres.= ");
  u8x8.println(bme.readPressure() / 100.0F);
  u8x8.println(" hPa");

  u8x8.print("Alt.= ");
  u8x8.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  u8x8.println(" m");

  u8x8.print("Hum.= ");
  u8x8.print(bme.readHumidity());
  u8x8.println(" %");
  
  u8x8.print("Light= ");
  u8x8.println(lux);
  
  u8x8.print("Moisture= ");
  u8x8.println (moisture_level);
//  u8x8.print("Version= "); 
//  u8x8.println(FuelGauge.version());
//  u8x8.print("ADC= "); 
//  u8x8.println(FuelGauge.adc());
//  u8x8.print("Voltage= "); 
//  u8x8.print(FuelGauge.voltage()); 
//  u8x8.println(" v");
//  u8x8.print("Percent= "); 
//  u8x8.print(FuelGauge.percent()); 
//  u8x8.println("%");


}

void loop()
{
  read_bme280();
  read_light();
  read_moisture();
  print_serial();
  print_screen();
  delay(1000);
}
