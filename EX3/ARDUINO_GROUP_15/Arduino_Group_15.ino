#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Pins Setup
#define buttonPin 3
#define ledPin 2
#define servoPin 9
#define BUZZER_PIN 8
#define SONAR_TRIGGER_PIN 12
#define SONAR_ECHO_PIN 13


// Variables
bool ledState = false; // Bulb status
bool buttonState = false;
bool lastButtonState = false;
Servo myservo;
unsigned int current_distance;
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD setup
const char* message = "Hello Guest";

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);

// LCD Initiallizing 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

 // Initial connection of the servo motor
  myservo.attach(servoPin);  
  myservo.write(90); // Resting the engine (at a 90 degree angle)

  
  // Settings for the distance sensor
  pinMode(SONAR_TRIGGER_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  handleButton();  // Button handling
  current_distance = measure_distance(); // measurement
  Serial.println(current_distance);      // Print to the serial screen
  delay(125);
// Handle the LCD display
  if (current_distance < 40) {
    lcd.clear();                       // clear the screen
    lcd.setCursor(0, 0);               // Set the cursor at the beginning of the line
    lcd.print(message);                // Print the message 
    tone(BUZZER_PIN, 540, 2000);
    delay(1000);
    noTone(BUZZER_PIN);
    Serial.print("current_distance: ");
    Serial.println(current_distance);
  } else {
    lcd.clear();                       // Clean the screen if the distance is greater than 40 cm

  }
}
void handleButton() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) { // pressing the button
    ledState = !ledState; // Change the bulb mode
    digitalWrite(ledPin, ledState);

    if (ledState) {
      myservo.attach(servoPin);  //Connect the engine to start working
      myservo.write(180); // Starting the engine to 180 degrees
    } else {
      myservo.detach();  // Engine cut-off (stops moving)
    }
  }
  lastButtonState = buttonState;
}

unsigned int measure_distance() {
  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIGGER_PIN, LOW);

  unsigned long pulse_length = pulseIn(SONAR_ECHO_PIN, HIGH); // Measuring the pulse time
  return (unsigned int)(pulse_length * 0.034 / 2);           // Convert to distance in cm
}