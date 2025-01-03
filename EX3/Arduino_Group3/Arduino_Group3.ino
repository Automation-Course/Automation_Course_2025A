#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <IRremote.h>

// Pin Definitions
#define buttonPin 3
#define ledPin 7
#define servoPin 6
#define RST_PIN 9
#define SS_PIN 10
#define IR_RECEIVER_PIN 2

// Global Variables
bool help = false;
bool plateDelivered = false;
bool sensor = false;
bool systemActive = false;
int angle = 45;

// Timer
unsigned long startTime;
const unsigned long deliveryTimeout = 120000;

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Servo Setup
Servo servoMotor;

// RFID Setup
MFRC522 rfid(SS_PIN, RST_PIN);

// IR Receiver Setup
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Pin Modes
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize Servo
  servoMotor.attach(servoPin);
  servoMotor.write(90); // Initial position

  // Initialize SPI and RFID
  SPI.begin();
  rfid.PCD_Init();

  // Initialize IR Receiver
  irrecv.enableIRIn();

  // Startup Message
  lcd.setCursor(0, 0);
  lcd.print("System Readyu");
  delay(2000);
  lcd.clear();

  // Timer Initialization
  startTime = millis();
}

void loop() {
  // Check for IR signal
  if (irrecv.decode(&results)) {
    if (results.value == 0xFFA25D) { // "FFA25D" to activate the system
      systemActive = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("System Activated");
      delay(2000);
      lcd.clear();
          // Reset global variables
       plateDelivered = false;
        sensor = false;
        startTime = millis();
       help = false;

    } else if (results.value == 0xFF629D) { // "FF629D" to deactivate the system
      systemActive = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("System Deactivated");
      digitalWrite(ledPin, LOW); // Turn off LED
      servoMotor.write(90); // Reset servo position
      delay(2000);
      lcd.clear();
    }
    irrecv.resume();
  }

  // If system is not active, skip the rest of the code
  if (systemActive) {
  
  

  // Check for button press to reset
  if (digitalRead(buttonPin) == HIGH) {
    help = true;
  }

  if (help) {
    // Reset global variables
    plateDelivered = false;
    sensor = false;
    startTime = millis();

    // Reinitialize RFID
    rfid.PCD_Init();

    // Display reset message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Timer Reset");
    digitalWrite(ledPin, LOW); // Turn off LED
    delay(2000); // Show reset message for 2 seconds
    servoMotor.write(90); // Reset servo position
    lcd.clear();

    help = false;
  }

  // Check for RFID card
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial() && !plateDelivered) {
    plateDelivered = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Table X:");
    lcd.setCursor(0, 1);
    lcd.print("Plate Delivered!");
    servoMotor.write(90); // Servo action
    delay(5000); // Show message for 5 seconds
    lcd.clear();
    sensor = true;

    // Halt RFID reading for this card
    rfid.PICC_HaltA();
  }

  // Check if delivery is late
  if (!plateDelivered && (millis() - startTime >= deliveryTimeout)) {
    lcd.setCursor(0, 0);
    lcd.print("Table X:");
    lcd.setCursor(0, 1);
    lcd.print("Delivery Late!");
    digitalWrite(ledPin, HIGH); // Turn on LED
    servoMotor.write(angle); // Alert position
    delay(2000);
    angle = 180 - angle;
    lcd.clear();
    sensor = false;
  }
  }
}
