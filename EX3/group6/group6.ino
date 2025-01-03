#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// DS18B20 - Temperature Sensor
#define ONE_WIRE_BUS A0 // DS18B20 data pin connected to A0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperature;

// LEDs
const int ledPin = 8;        // Empty wipe LED
const int low_temp = 4;      // LED to simulate hitting the device
const int high_temp = 12;    // LED to simulate hitting the device

int counterwipe = 0;         // Counter for used wipes

// Servo motor
Servo myServo;
int position = 0;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set I2C address (adjust as needed)

// Ultrasonic
long duration;  // Ultrasonic duration
float distance; // Calculated distance

const int triggerPin = 6; // Ultrasonic sensor trigger pin
const int echoPin = 7;    // Ultrasonic sensor echo pin

// Button
const int buttonPin = 2;  // Button pin
int buttonState = 0;      // Current button state
int lastButtonState = HIGH; // Previous button state (HIGH = not pressed)
int temperatureState = -1;  // 0 = No Temp, 1 = Low Temp, 2 = High Temp
bool printTempState = false; // Flag to ensure one-time print per button press

void setup() {
  // LEDs
  pinMode(ledPin, OUTPUT);
  pinMode(low_temp, OUTPUT);
  pinMode(high_temp, OUTPUT);

  // Servo
  myServo.attach(9);  // Servo connected to pin 9
  myServo.write(0);   // Ensure servo starts in the neutral position
  
  delay(925);
  myServo.write(90);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.print("choose temperature");

  // Configure pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Button with internal pull-up resistor

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("System Initialized.");
}

void loop() {
  // Read temperature from DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  temperature = sensors.getTempCByIndex(0); // Read the first sensor's temperature
  
  // Button Handling
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW && lastButtonState == HIGH) {
    Serial.println("Button Pressed!");
    // Update temperature state
    temperatureState++;
    if (temperatureState > 2) {
      temperatureState = 0;
    }
    printTempState = true; // Enable print flag
    delay(100);            // Debounce delay
  }

  if (printTempState) {
    Serial.println("The temp is:");
    Serial.println(temperature);
    // Display temperature state on LCD
    lcd.clear();  // Clear LCD before printing new state
    if (temperatureState == 0) {
      lcd.print("Room Temp");
      Serial.println("Room Temperature Selected");
    } else if (temperatureState == 1) {
      lcd.print("Low Temp: 15 C");
      Serial.println("Low Temperature Selected");
      if (temperature < 15) {
        digitalWrite(low_temp, HIGH);
        delay(5000);
        digitalWrite(low_temp, LOW);
        delay(200);
        Serial.println("Low Temp: Small action.");
      }
    } else if (temperatureState == 2) {
      lcd.print("High Temp: 30 C");
      Serial.println("High Temperature Selected");
      if (temperature < 30) {
        digitalWrite(high_temp, HIGH);
        delay(5000);
        digitalWrite(high_temp, LOW);
        delay(200);
        Serial.println("High Temp: Strong action.");
      }
    }
    printTempState = false; // Reset flag
  }

  lastButtonState = buttonState; // Update button state

  // Ultrasonic Sensor
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Measure echo duration
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance (cm)
  distance = (duration * 0.0343) / 2;

  if (distance < 5 && counterwipe < 4) {
    Serial.println("Client detected!");
    Serial.print("Distance: ");
    Serial.println(distance);
   
    // Servo action to simulate dispensing the wipe
    myServo.write(0); // Dispense wipe
    delay(925); // Wait for the servo to move to the desired position

    myServo.write(90); // Return to neutral position
    delay(200); // Wait for servo to return to neutral position
    counterwipe++;
    Serial.print("Wipe used: ");
    Serial.println(counterwipe);
  }

  // If all wipes are used, turn on the empty wipe LED
  if (counterwipe >= 4) {
    digitalWrite(ledPin, HIGH); // Out of wipes
  }

  delay(100); // Avoid overlapping measurements
}
