#include <algorithm>
#include <iostream>
#include <WiFi.h>
#include <Button2.h>            //https://github.com/LennartHennigs/Button2
#include <BH1750.h>             //https://github.com/claws/BH1750
#include <DHT12.h>              //https://github.com/xreef/DHT12_sensor_library
#include "ds18b20.h"
#include "configuration.h"

#ifdef __HAS_BME280__
#include <Adafruit_BME280.h>
#endif

#ifdef __HAS_SHT3X__
#include "SHT3X.h"
#endif

#ifdef __HAS_MOTOR__
#include <Adafruit_NeoPixel.h>
#endif

// #include <NTPClient.h>
#include <ThingSpeak.h>

// ThingSpeak
unsigned long myChannelNumber = MY_CHANNEL_NUMBER;  // Replace the 0 with your channel number
const char * myWriteAPIKey = MY_WRITE_API_KEY;    // Paste your ThingSpeak Write API Key between the quotes
// [Create the Thingspeak client instance]
WiFiClient client;

//uint32_t timestamp;     /**< time is in milliseconds */
float temperature;      /**< temperature is in degrees centigrade (Celsius) */
float light;            /**< light in SI lux units */
float pressure;         /**< pressure in hectopascal (hPa) */
float humidity;         /**<  humidity in percent */
float altitude;         /**<  altitude in m */
float voltage;           /**< voltage in volts (V) */
uint8_t soil;           //Percentage of soil
uint8_t salt;           //Percentage of salt
uint16_t soilMoisture;

BH1750              lightMeter(OB_BH1750_ADDRESS);  //0x23
DHT12               dht12(DHT12_PIN, true);
Button2             button(BOOT_PIN);
Button2             useButton(USER_BUTTON);

#ifdef __HAS_DS18B20__
DS18B20             dsSensor(DS18B20_PIN);
#endif /*__HAS_DS18B20__*/

#ifdef __HAS_SHT3X__
SHT3X               sht30;
#endif /*__HAS_SHT3X__*/

#ifdef __HAS_BME280__
Adafruit_BME280     bme;                            //0x77
#endif /*__HAS_BME280__*/

#ifdef __HAS_MOTOR__
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
#endif  /*__HAS_MOTOR__*/


bool                has_bmeSensor   = true;
bool                has_lightSensor = true;
bool                has_dhtSensor   = true;
uint64_t            timestamp       = 0;


void deviceProbe(TwoWire &t);

void sleepHandler()
{
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  	Serial.println("Sleeping for " + String(TIME_TO_SLEEP) +" Seconds");
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    // delay(1000);
    esp_deep_sleep_start();
}

void setupWiFi()
{
#ifdef SOFTAP_MODE
    Serial.println("Configuring access point...");
    uint8_t mac[6];
    char buff[128];
    WiFi.macAddress(mac);
    sprintf(buff, "T-Higrow-%02X:%02X", mac[4], mac[5]);
    WiFi.softAP(buff);
    Serial.printf("The hotspot has been established, please connect to the %s and output 192.168.4.1 in the browser to access the data page \n", buff);
#else
    WiFi.mode(WIFI_STA);

    Serial.print("Connect SSID:");
    Serial.print(WIFI_SSID);
    Serial.print(" Password:");
    Serial.println(WIFI_PASSWD);

    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi connect fail!");
        delay(3000);
        esp_restart();
    }
    Serial.print("WiFi connect success ! , ");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
#endif
}

void read_sensors()
{

        // read bme280 (if enabled and installed)
#ifdef __HAS_BME280__
        if (has_bmeSensor) {
            temperature = bme.readTemperature();
            pressure = (bme.readPressure() / 100.0F);
            humidity = bme.readHumidity();
            altitude = bme.readAltitude(1029);
        }
#endif

        // read sht3x (if enabled and installed)
#ifdef __HAS_SHT3X__
        if (has_dhtSensor) {
            if (sht30.get()) {
                temperature = sht30.cTemp;
                humidity = sht30.humidity;
            }
    }
#endif
        // read dht12 (if bme280 not installed)
        if (!has_bmeSensor) {
        temperature = dht12.readTemperature();
        humidity = dht12.readHumidity();
          if (isnan(temperature)) {
              temperature = 0.0;
          }
          if (isnan(humidity)) {
              humidity = 0.0;
          }
        }
        // Read ligh level
        if (has_lightSensor) {
            light = lightMeter.readLightLevel();
        } else {
            light = 0;
        }
        // Read soil level
        soilMoisture = analogRead(SOIL_PIN);
        soil = map(soilMoisture, soilMoistureMin, soilMoistureMax, 100, 0);

        // read salt level
        uint8_t samples = 120;
        uint32_t humi = 0;
        uint16_t array[120];
        for (int i = 0; i < samples; i++) {
            array[i] = analogRead(SALT_PIN);
            delay(2);
        }
        std::sort(array, array + samples);
        for (int i = 1; i < samples - 1; i++) {
            humi += array[i];
        }
        humi /= samples - 2;
        salt = humi;
#ifdef __HAS_DS18B20__ 
        // read ds18b20 (if enabled and installed)
        ds18b20temperature = dsSensor.temp();
        if (isnan(ds18b20temperature) || ds18b20temperature > 125.0) {
            ds18b20temperature = 0;
        }
#endif
        // read battery voltage
        int vref = 1100;
        uint16_t volt = analogRead(BAT_ADC);
        voltage = ((float)volt / 4095.0) * 6.6 * (vref);
}

void setup()
{
    Serial.begin(115200);

    //! Sensor power control pin , use deteced must set high
    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, 1);
    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);

    deviceProbe(Wire);

    dht12.begin();

    if (!lightMeter.begin()) {
        Serial.println(F("Could not find a valid BH1750 sensor, check wiring!"));
        has_lightSensor = false;
    }

#ifdef __HAS_BME280__
    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        has_bmeSensor = false;
    }
#endif /*__HAS_BME280__*/

#ifdef __HAS_SHT3X__
    Wire1.begin(21, 22);
    deviceProbe(Wire1);
#endif /*__HAS_SHT3X__*/

read_sensors();

Serial.println("Sensor readings:");
#ifdef __HAS_BME280__
Serial.println("--------------BME280--------------");
Serial.println("Air Temperature = " + String(temperature) + " *C");
Serial.println("Air Pressure = " + String(pressure) + " hPa");
Serial.println("Approx. Altitude = " + String(altitude) + " m");
Serial.println("Air Humidity = " + String(humidity) + " %");
#endif /*__HAS_BME280__*/
#ifdef __HAS_DS18B20__ 
Serial.println("--------------DS18B20-------------");
Serial.println("Soil Temperature = " + String(ds18b20temperature) + " *C");
#endif /*__HAS_DS18B20__*/
Serial.println("--------------BATTERY-------------");
Serial.println("Battery Voltage = " + String(voltage) + " V");
Serial.println("--------------SOIL----------------");
Serial.println("Water level = " + String(soil) + " %");
Serial.println("Water level raw = " + String(soilMoisture));
Serial.println("Salt level = " + String(salt) + " %");
Serial.println("--------------BH1750--------------");
Serial.println("Light level = " + String(light) + " Lux");

    //setupWiFi();
    //uploadSensorReading();
    sleepHandler();
}

void uploadSensorReading()
{

  ThingSpeak.begin(client);
  ThingSpeak.setField(1, light);
  ThingSpeak.setField(2, temperature);
  ThingSpeak.setField(3, humidity);
  ThingSpeak.setField(4, soil);
  ThingSpeak.setField(5, voltage);
  ThingSpeak.setField(6, pressure);
  ThingSpeak.setField(7, salt);
  ThingSpeak.setField(8, altitude);

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

void deviceProbe(TwoWire &t)
{
    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        t.beginTransmission(addr);
        err = t.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            switch (addr) {
            case OB_BH1750_ADDRESS:
                has_dhtSensor = true;
                break;
            case OB_BME280_ADDRESS:
                has_bmeSensor = true;
                break;
            case OB_SHT3X_ADDRESS:
                has_dhtSensor = true;
                break;
            default:
                break;
            }
            nDevices++;
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}


void loop() {
//  // Reading Capacitive Soil value
//  sensorValue = analogRead(sensorPin);
//  // Sensor value has been meased to be 1400 when completely wet and 3400 when dry
//  sensorMappedValue = map(sensorValue, 1400, 3400, 0, 100);
//  Serial.println(sensorMappedValue);
//  delay(500);
}
