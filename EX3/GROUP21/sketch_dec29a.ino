#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <IRremote.h>

// הגדרות עבור מסך LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// הגדרות חיישן מרחק
const int trigPin = 4;
const int echoPin = 5;

// הגדרות LED
const int redLED = 7;
const int yellowLED = 8;
const int greenLED = 12;

// הגדרות כפתור
const int buttonPin = 6;

// הגדרות מקלט IR
const int irPin = 9;
IRrecv irrecv(irPin);
decode_results results;

// הגדרות מנוע סרוו
Servo servoMotor;
const int servoPin = 10;

// משתנים
long duration;
int distance;
bool isCustomerDetected = false;
bool isOptionSelected = false;
bool isMotorActivated = false;
bool isButtonPressed = false;
bool isMotorCompleted = false;
unsigned long motorStartTime = 0;
unsigned long yellowLEDStartTime = 0;
const unsigned long motorRunDuration = 10000; // 10 שניות

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  lcd.setCursor(0, 1);
  lcd.print("Please Wait...");
  delay(2000);
  lcd.clear();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buttonPin, INPUT);

  servoMotor.attach(servoPin);

  irrecv.enableIRIn();

  Serial.begin(9600);
}

void loop() {
  if (!isCustomerDetected) {
    detectCustomer();
  } else if (!isOptionSelected) {
    processIRInput();
  } else if (!isMotorCompleted) {
    activateMotor();
  } else if (!isButtonPressed) {
    processButtonPress();
  }
}

void detectCustomer() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 60000);
  distance = (duration > 0) ? (duration * 0.034 / 2) : -1;

  if (distance > 0 && distance <= 50) {
    isCustomerDetected = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Welcome Customer");
    lcd.setCursor(0, 1);
    lcd.print("Please Proceed");
    blinkLED(redLED, 2000);
  }
}

void processIRInput() {
  if (irrecv.decode(&results)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Option Selected:");
    lcd.setCursor(0, 1);
    lcd.print(results.value, HEX);
    blinkLED(yellowLED, 500);
    yellowLEDStartTime = millis();
    isOptionSelected = true;
    irrecv.resume();
  }
}

void activateMotor() {
  unsigned long currentMillis = millis();

  // חכה עד שהנורה הצהובה תכבה לפני הפעלת המנוע
  if (currentMillis - yellowLEDStartTime >= 500 && motorStartTime == 0) {
    motorStartTime = currentMillis;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Running...");
    servoMotor.write(90); // הפעלת המנוע בזווית כלשהי
  }

  // הפעל את המנוע למשך 10 שניות
  if (motorStartTime > 0 && currentMillis - motorStartTime >= motorRunDuration) {
    servoMotor.write(0); // עצירת המנוע
    motorStartTime = 0;
    isMotorCompleted = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("serving...");
    lcd.setCursor(0, 1);
    lcd.print("Ready Again");
  }
}

void processButtonPress() {
  if (digitalRead(buttonPin) == HIGH) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calling Waiter");
    blinkLED(greenLED, 5000);
    isButtonPressed = true;
    resetSystem();
  }
}

void resetSystem() {
  isCustomerDetected = false;
  isOptionSelected = false;
  isMotorActivated = false;
  isButtonPressed = false;
  isMotorCompleted = false;
  motorStartTime = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Reset");
  lcd.setCursor(0, 1);
  lcd.print("Ready to Use");
  delay(2000);
}

void blinkLED(int pin, int duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}