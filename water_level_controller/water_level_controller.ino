int Moisture_Level_Sensor = A0; // select the input pin for the Moisture Sensor
int Moisture_Level_Power = D3;  // select the pin for the Moisture Sensor Power
int Motor_Power_Pin = D5;       // select the pin for the Water_Motor
int Sleep_Led = D4;             // Display sleep mode
// int Moisture_Level_Sensor = 0;  // variable to store the value coming from the sensor

int Min_Moisture_Level = 300; // Moisture Level of dry soil
int Max_Moisture_Level = 700; // Moisture Level of wet soil
int Max_Watering_Time = 10000; // 10 Seconds

int Time_Measurement;
int First_Moisture_Level;
int Start_Watering_Time;

void setup() {
  // Set up pin modes:
  pinMode(Moisture_Level_Power, OUTPUT);
  digitalWrite(Moisture_Level_Power, LOW);  
  pinMode(Motor_Power_Pin, OUTPUT);
  digitalWrite(Motor_Power_Pin, LOW);  
  pinMode(Sleep_Led, OUTPUT);
  digitalWrite(Motor_Power_Pin, HIGH);  
  Serial.begin(9600);
}

void loop() {
  
  int Current_Moisture_Level = 0;
  int Current_Watering_Time = 0;

  // Serial.println("Start of program");

  if (digitalRead(Sleep_Led) == LOW) {
    digitalWrite(Sleep_Led, HIGH); // Turn off sleep led
  }


  Time_Measurement = millis();
  if (digitalRead(Moisture_Level_Power) == LOW) {
    digitalWrite(Moisture_Level_Power, HIGH); // Turn on moisture sensor to make a reading
  }
  First_Moisture_Level = analogRead(Moisture_Level_Sensor);
  // Serial.print("First Moisture Level: "); Serial.println(First_Moisture_Level);
  if (First_Moisture_Level <= Min_Moisture_Level) {
    Serial.println("Plant is dry, watering plant");
    Start_Watering_Time = millis();
    while ((Current_Moisture_Level <= Max_Moisture_Level) && (Current_Watering_Time <= Max_Watering_Time)) {
      Current_Watering_Time = millis() - Start_Watering_Time;
      Current_Moisture_Level = analogRead(Moisture_Level_Sensor);
      // Serial.print("Current Moisture Level: "); Serial.println(Current_Moisture_Level);
      // Serial.print("Max Moisture Level: "); Serial.println(Max_Moisture_Level);
      if (digitalRead(Motor_Power_Pin) == LOW) {
        digitalWrite(Motor_Power_Pin, HIGH);
      }
      delay(10);
    }
  }
  Current_Moisture_Level = analogRead(Moisture_Level_Sensor);
  if (digitalRead(Moisture_Level_Power) == HIGH) {
    digitalWrite(Moisture_Level_Power, LOW); // Make sure the moisture sensor is powered down.
  }
  if (digitalRead(Motor_Power_Pin) == HIGH) {
    digitalWrite(Motor_Power_Pin, LOW); // Make sure the watering motor is powered down.
  }
  if (First_Moisture_Level <= Min_Moisture_Level) {
    // Serial.println("Plant needed watering");
    Serial.print("Moisture Level change: "); Serial.println(Current_Moisture_Level - First_Moisture_Level);  
    Serial.print("Watering time: "); Serial.println(Current_Watering_Time);
  }
  if ((First_Moisture_Level > Min_Moisture_Level) && (First_Moisture_Level <= (Max_Moisture_Level+50))) {
    Serial.println("Plant is happy");
  }
  if (First_Moisture_Level > (Max_Moisture_Level+50)) {
    Serial.println("Plant is too wet");
  }

  // Serial.print("Execute time: "); Serial.println(millis() - Time_Measurement);

  if (digitalRead(Sleep_Led) == HIGH) {
    digitalWrite(Sleep_Led, LOW); // Turn off sleep led
  }
  Serial.print("Restarting in: 5");
  delay(1000);  
  Serial.print("..4");
  delay(1000);  
  Serial.print("..3");
  delay(1000);  
  Serial.print("..2");
  delay(1000);  
  Serial.println("..1");
  delay(1000);
}
