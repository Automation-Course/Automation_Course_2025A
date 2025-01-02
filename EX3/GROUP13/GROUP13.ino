#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// *** Component Definitions ***
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD I2C address
Servo myServo;

// *** Pin Definitions ***
const int pirPin = 7;         // PIR sensor
const int photoresistorPin = A0; // Photoresistor
const int servoPin = 9;       // Servo motor pin
const int ledPin = 13;        // LED pin
const int buttonPin = 2;      // Button pin

// *** Variables ***
const int lightThreshold = 500; // Light intensity threshold
bool motionDetected = false;    // Tracks motion detection
bool systemActive = false;      // Tracks if the system is active
bool servoCompleted = false;    // Tracks if the servo action is complete
bool motionHandled = false;     // Prevents duplicate motion detection
int buttonPressCount = 0;       // Tracks button press count

// *** Setup Function ***
void setup() {
  lcd.init();
  lcd.backlight();
  myServo.attach(servoPin);
  myServo.write(90); // Initial position: stop

  pinMode(pirPin, INPUT);
  pinMode(photoresistorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600); // Initialize Serial Monitor
  displayWelcomeMessage();
  Serial.println("Servo initialized and set to 90 degrees.");
}

// *** Main Loop ***
void loop() {
  if (detectMotion() && !systemActive) { 
    activateSystem(); 
  }

  if (checkButton() && systemActive) {
    handleButtonPress();
  }
}

// *** Helper Functions ***

// Display welcome message on LCD
void displayWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BGU Michelin Rest");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for Guest");
  Serial.println("System in standby mode - waiting for a guest");
}

// Detect motion using the PIR sensor
bool detectMotion() {
  int motionState = digitalRead(pirPin);
  if (motionState == HIGH && !motionHandled) {
    motionDetected = true;
    motionHandled = true; // Mark motion as handled
    Serial.println("Motion detected! A guest is approaching.");
    return true;
  } else if (motionState == LOW) {
    motionHandled = false; // Reset when no motion is detected
  }
  return false;
}

// Activate the system
void activateSystem() {
  systemActive = true;

  // Detect light level and display the appropriate greeting
  int lightLevel = analogRead(photoresistorPin);
  lcd.clear();
  lcd.setCursor(0, 0);

  if (lightLevel > lightThreshold) {
    lcd.print("Good Morning!");
    Serial.print("Light level: ");
    Serial.println(lightLevel);
    Serial.println("Displaying 'Good Morning!'");
  } else {
    lcd.print("Good Evening!");
    Serial.print("Light level: ");
    Serial.println(lightLevel);
    Serial.println("Displaying 'Good Evening!'");
  }

  lcd.setCursor(0, 1);
  lcd.print("Welcome!");
  Serial.println("Waiting for button press to confirm seating.");

  // Turn on LED
  digitalWrite(ledPin, HIGH);
  Serial.println("LED turned on.");

  // Move servo
  if (!servoCompleted) {
    Serial.println("Starting servo movement.");
    myServo.write(360); // Move servo to 360 degrees
    delay(5000);        // Wait for 5 second
    myServo.write(90);  // Return servo to 90 degrees (stop position)
    servoCompleted = true;
    Serial.println("Servo movement completed.");
  }
}

// Check if the button is pressed
bool checkButton() {
  if (digitalRead(buttonPin) == LOW) {
    delay(200); // Prevent bouncing
    return true;
  }
  return false;
}

// Handle button presses
void handleButtonPress() {
  buttonPressCount++;

  if (buttonPressCount == 1) {
    // First press: Guest confirmed seating
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enjoy your meal!");
    Serial.println("Guest confirmed seating.");
  } else if (buttonPressCount == 2) {
    // Second press: Guest finished meal
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Thank you!");
    lcd.setCursor(0, 1);
    lcd.print("Hope you enjoyed");
    Serial.println("Guest finished, displaying thank you message.");
    delay(4000); // Display message for 4 seconds
    resetSystem(); // Reset the system
  }
}

// Reset the system
void resetSystem() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Resetting");
  Serial.println("Resetting the system...");
  delay(2000);

  // Turn off LED
  digitalWrite(ledPin, LOW);
  Serial.println("LED turned off.");

  // Return to initial state
  displayWelcomeMessage();
  systemActive = false;
  motionDetected = false;
  servoCompleted = false;
  motionHandled = false; // Reset motion handling
  buttonPressCount = 0;  // Reset button press count
  Serial.println("System returned to standby mode.");
}
