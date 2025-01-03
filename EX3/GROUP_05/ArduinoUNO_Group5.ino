#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// Initialize Components
Servo servo;
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD address, columns, rows

// Pin Definitions
const int ultrasonicTrig = 5;
const int ultrasonicEcho = 6;
const int buttonPin = 11;
const int buzzerPin = 13;
const int lightPin = 4;
const int servoPin = 3;

// Variables
bool buzzerActivated = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("System Initializing...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize Servo
  servo.attach(servoPin);
  servo.write(90); // Start position (closed)
  Serial.println("System is ready");

  // Pin Modes
  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  Serial.println("All pins configured successfully.");

  // Display "Welcome!" message
  lcd.setCursor(0, 0);
  lcd.print("Welcome!");
  Serial.println("Displaying welcome message on LCD for 2 seconds.");
  delay(2000); // Display "Welcome!" for 2 seconds
  lcd.clear();
}

void loop() {
  // Detect proximity using the ultrasonic sensor
  if (detectProximity(5)) {
    Serial.println("Proximity detected! Object within 5 cm.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Find Button");
    Serial.println("Displayed 'Find Button' message on LCD.");

    // Activate buzzer and light
    playMusicAndFlashLight();
    buzzerActivated = true;
    Serial.println("Buzzer and light deactivated.");

    // Wait for the button press
    if (waitForButtonPress()) {
      if (buzzerActivated) {
        Serial.println("Button pressed. ");
        // Rotate servo to open
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Be Te A Von");
        Serial.println("Displayed 'Be Te A Von' message on LCD.");
        servo.write(0); // Rotate servo 90 degrees (open position)
        Serial.println("Opening the Door (activate Servo).");
        delay(5000);    // Keep it open for 5 seconds

        // Reset servo and state
        servo.write(90); // Return servo to closed position
        Serial.println("Door closed (servo stopped");
        buzzerActivated = false;

        // Clear LCD for the next interaction
        lcd.clear();
        Serial.println("System reset for next interaction.");
      }
    }
  }
}

// Function to detect proximity using ultrasonic sensor
// Function to detect proximity using ultrasonic sensor
bool detectProximity(int targetDistance) {
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);

  long duration = pulseIn(ultrasonicEcho, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm

  // Print distance to Serial Monitor only when it's 5 or less
  if (distance <= targetDistance) {
    Serial.print("Proximity detected! Distance: ");
    Serial.print(distance);
    Serial.println(" cm.");
    return true;
  }

  // No action if distance is greater than target
  return false;
}


// Function to play music and flash light
void playMusicAndFlashLight() {
  Serial.println("Flashing light and playing buzzer for 5 seconds.");
  for (int i = 0; i < 5; i++) { // 5 seconds of flashing
    digitalWrite(lightPin, HIGH);
    tone(buzzerPin, 1000); // Play tone
    delay(500);
    digitalWrite(lightPin, LOW);
    noTone(buzzerPin); // Stop tone
    delay(500);
  }
  Serial.println("Light and buzzer sequence complete.");
}

// Function to wait for button press
bool waitForButtonPress() {
  Serial.println("Waiting for button press...");
  while (true) {
    if (digitalRead(buttonPin) == LOW) {
      delay(200); // Debounce
      Serial.println("Button pressed!");
      return true;
    }
  }
}
