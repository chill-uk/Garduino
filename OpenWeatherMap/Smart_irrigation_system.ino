# Copied from https://create.arduino.cc/projecthub/officine/getting-weather-data-655720

#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFi101.h>
#include <I2CSoilMoistureSensor.h>


char ssid[] = SECRET_SSID; //  your network SSID (name)
char pass[] = SECRET_PSW;//  your network PASSWORD ()

//open weather map api key
String apiKey= SECRET_APIKEY;

//the city you want the weather for
String location= "torino,IT";

int status = WL_IDLE_STATUS;
char server[] = "api.openweathermap.org";    

WiFiClient client;

I2CSoilMoistureSensor soilMoistureSensor(0x60);
float soilTemperature;
int soilMoisture;

String nextWeatherTime[]={" "," "," "};
String nextWeather[] ={" "," "," "};

int relay_pin=13;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  pinMode(relay_pin,OUTPUT);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    
    status = WiFi.begin(ssid);
    //use the line below if your network is protected by wpa password 
    //status = WiFi.begin(ssid, pass);
    
    // wait 10 seconds for connection:
    delay(1000);
  }
  Serial.println("Connected to wifi");
  
   soilMoistureSensor.begin(); // reset sensor
  delay(1000); // give some time to boot up
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(soilMoistureSensor.getAddress(),HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(soilMoistureSensor.getVersion(),HEX);
  Serial.println();
}

void loop() {
  getWeather();
  delay(10000);
  
  ///////////////////////////////////////////////
  //get all the data from the soul moisture sensor
  ///////////////////////////////////////////////
  while (soilMoistureSensor.isBusy()) delay(50); // available since FW 2.3
  soilMoisture=soilMoistureSensor.getCapacitance(); 
  soilTemperature=soilMoistureSensor.getTemperature()/(float)10; 
  soilMoistureSensor.sleep(); // available since FW 2.3
  
  
  ///////////////////////////////////////////////
  //verify the conditions.
  ///////////////////////////////////////////////
  
  if (soilMoisture<500){
    Serial.println("I feel a bit dry");
  
    Serial.println("checking weather conditions");
  
    if (nextWeather[0]=="rain"||nextWeather[1]=="rain"||nextWeather[2]=="rain"){
      Serial.println("i see clouds at the horizon!");
    }else{
      Serial.println("better open the water here");
      digitalWrite(relay_pin, HIGH);
    }
    
  }else{
    Serial.println("it's nice and humid down here");
  }
  
}


void getWeather() {

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.print("GET /data/2.5/forecast?");
    client.print("q="+location);
    client.print("&appid="+apiKey);
    client.print("&cnt=3");
    client.println("&units=metric");
    client.println("Host: api.openweathermap.org");
    client.println("Connection: close");
    client.println();
  } else {
    Serial.println("unable to connect");
  }

  delay(1000);
  String line = "";

  while (client.connected()) {
    line = client.readStringUntil('\n');
    
    //Serial.println(line);
    Serial.println("parsingValues");

    //create a json buffer where to store the json data
    StaticJsonBuffer<5000> jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(line);
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }

  //get the data from the json tree
  String nextWeatherTime0 = root["list"][0]["dt_txt"];
  String nextWeather0 = root["list"][0]["weather"][0]["main"];

  String nextWeatherTime1 = root["list"][1]["dt_txt"];
  String nextWeather1 = root["list"][1]["weather"][0]["main"];
  
  String nextWeatherTime2 = root["list"][2]["dt_txt"];
  String nextWeather2 = root["list"][2]["weather"][0]["main"];
  
  nextWeatherTime[0]=nextWeatherTime0;
  nextWeatherTime[1]=nextWeatherTime0;
  nextWeatherTime[2]=nextWeatherTime0;

  nextWeather[0]=nextWeather0;
  nextWeather[1]=nextWeather1;
  nextWeather[2]=nextWeather2;

  // Print values.
  Serial.println(nextWeatherTime0);
  Serial.println(nextWeather0);
  Serial.println(nextWeatherTime1);
  Serial.println(nextWeather1);  
  Serial.println(nextWeatherTime2);
  Serial.println(nextWeather2);

  }
}