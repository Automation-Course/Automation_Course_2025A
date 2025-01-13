// Full Arduino Project Code for Interactive Restaurant System
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Sensors and Actuators
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
Servo servoMotor;
const int servoPin = 9;
const int trigPin = 3;
const int echoPin = 4;
const int buttonPin = 5;

// LEDs
const int greenLedPin = 6;  // Presence detected
const int yellowLedPin = 7; // Cooling
const int redLedPin = 8;    // Heating

// Variables
long duration;
int distance;
float temperatureC;
unsigned long previousMillis = 0;
unsigned long tempCheckMillis = 0;
unsigned long servoMoveMillis = 0;
int buttonState = 0;
int lastButtonState = 0;
int mealStage = 0;
int tempSampleCount = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // Initialize DHT Sensor and Servo
  dht.begin();
  servoMotor.attach(servoPin);
}

void loop() {
  unsigned long currentMillis = millis();

  // Stage 0: Servo scanning until presence detected
  if (mealStage == 0) {
    scanForPresence(currentMillis);
  }

  // Stage 1-3: Meal stages
  else if (mealStage >= 1 && mealStage <= 3) {
    handleMealStage(currentMillis);
  }

  // Stage 4: Goodbye message
  else if (mealStage == 4) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Goodbye");
    lcd.setCursor(0, 1);
    lcd.print("Thanks!");
  }

  // Button handling for stage advancement
  handleButtonPress();
}

void scanForPresence(unsigned long currentMillis) {
  if (currentMillis - servoMoveMillis >= 100) {
    static int angle = 0;
    servoMotor.write(angle);
    angle += 10;
    if (angle > 180) angle = 0;
    servoMoveMillis = currentMillis;
  }

  // Measure distance every 5 seconds
  if (currentMillis - previousMillis >= 5000) {
    measureDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    previousMillis = currentMillis;
  }

  if (distance > 0 && distance <= 50) {
    digitalWrite(greenLedPin, HIGH);
    servoMotor.detach();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Welcome!");
    mealStage = 1;
  }
}

void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}

void handleMealStage(unsigned long currentMillis) {
  float requiredTemp = 0;
  String mealName;

  if (mealStage == 1) {
    requiredTemp = 24;
    mealName = "First Course";
  } else if (mealStage == 2) {
    requiredTemp = 22;
    mealName = "Second Course";
  } else if (mealStage == 3) {
    requiredTemp = 16;
    mealName = "Third Course";
  }

  if (currentMillis - tempCheckMillis >= 1000) {
    temperatureC = dht.readTemperature();
    tempSampleCount++;
    if (tempSampleCount >= 2) {
      Serial.print(mealName);
      Serial.print(" : Room Temp = ");
      Serial.println(temperatureC);
      tempSampleCount = 0;
    }
    tempCheckMillis = currentMillis;
  }

  lcd.setCursor(0, 0);
  lcd.print(mealName);
  lcd.setCursor(0, 1);
  lcd.print("Req Temp: ");
  lcd.print(requiredTemp);

  if (temperatureC < requiredTemp) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
  } else {
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
  }
}

void handleButtonPress() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    mealStage++;
    if (mealStage > 4) {
      mealStage = 0;
      servoMotor.attach(servoPin);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      lcd.clear();
    }
  }
  lastButtonState = buttonState;
}