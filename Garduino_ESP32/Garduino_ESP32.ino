// Capacitive Soil Sensor is connected to GPIO 34 (Analog ADC1_CH6) 
const int sensorPin = 34;

// variable for storing the sensor value
int sensorValue = 0;
int sensorMappedValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  sensorValue = analogRead(sensorPin);
  // Sensor value is 1400 when completely wet and 3400 when dry
  sensorMappedValue = map(sensorValue, 1400, 3400, 0, 100);
  Serial.println(sensorMappedValue);
  delay(500);
}
