#include <Servo.h>
#include <LiquidCrystal_I2C_Hangul.h>

// Pin definitions
#define LedPin 11
#define LdrPin A0
#define servoPin 9
#define PUSH_BUTTON 10

int LdrValue = 0;                       // Light sensor value
LiquidCrystal_I2C_Hangul lcd(0x27, 16, 2); // LCD object

// Servo control
int angle = 0;                          // Servo angle
bool Direction = true;                  // Servo direction
Servo servo;

unsigned int lastDebounceTime = 0;      // Debounce timer
unsigned int debounceDelay = 50;        // 50 ms debounce time

// Button state tracking
bool lastButtonState = HIGH;            // Start as HIGH (button not pressed)

// Define melody notes and durations
int melody[] = {
  262, 294, 330, 349, 392, 440, 494, 523 // C4, D4, E4, F4, G4, A4, B4, C5
};

int noteDuration[] = {
  400, 400, 400, 400, 400, 400, 400, 400 // Duration of each note in milliseconds
};

int buzzerPin = 2;                      // Buzzer pin

// For ultrasonic sensor
int trigPin = 6;
int echoPin = 5;

// For distance measurement
long duration, distance;

// For LCD handling
int lcdState = 0;                       // LCD update state
unsigned long lastLcdTime = 0;          // Timer for LCD updates

// Melody playback tracking
int noteIndex = 0;                      // Current note being played
unsigned long lastNoteTime = 0;         // Timer for melody playback
bool isPlayingMelody = false;           // Melody playback flag

// Distance state tracking
bool customerPresent = false;           // Tracks if a customer is within range

void setup() {
  Serial.begin(9600);
  pinMode(LdrPin, INPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin);
  pinMode(buzzerPin, OUTPUT);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);   // Enable internal pull-up resistor for button
}

void loop() {
  UltraSonic();                         // Trigger the ultrasonic sensor
  Distance_Calculation();               // Calculate distance
  handleDistanceState();                // Handle customer state changes
  handleButtonPress();                  // Check for button press
  lcd1();                               // Update LCD without blocking
  lightSensor();                        // Read light sensor without blocking

  // Handle distance-based actions only if melody is not playing
  if (!isPlayingMelody) {
    if (distance < 200) {
      servo1();                         // Move the servo
    } else {
      return_normal();                  // Reset to normal state
    }
  }
}

// Trigger the ultrasonic sensor
void UltraSonic() {
  digitalWrite(trigPin, LOW);           // Clear the trigPin
  delayMicroseconds(2);                 // Wait for a brief moment
  digitalWrite(trigPin, HIGH);          // Trigger the ultrasonic sensor
  delayMicroseconds(10);                // Wait for the trigger pulse
  digitalWrite(trigPin, LOW);           // Stop triggering
}

// Calculate distance based on ultrasonic sensor
void Distance_Calculation() {
  duration = pulseIn(echoPin, HIGH);    // Measure travel time of sound wave
  distance = duration * 0.034 / 2;      // Calculate distance in cm
}

// Handle customer state changes for event-driven printing
void handleDistanceState() {
  if (distance < 200 && !customerPresent) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm - A customer has arrived"); // Print distance and message
    customerPresent = true;                         // Update state to indicate a customer is present
  } else if (distance >= 200 && customerPresent) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm - No customers nearby");    // Print distance and message
    customerPresent = false;                        // Update state to indicate no customers are present
  }
}

// Non-blocking button press handling
void handleButtonPress() {
  bool currentButtonState = digitalRead(PUSH_BUTTON); // Read the current button state

  // Check for a button press (transition from HIGH to LOW)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("Waiter to the table");       // Print when the button is pressed
    isPlayingMelody = true;                      // Start the melody
    noteIndex = 0;                               // Reset melody to the first note
    lastNoteTime = millis();                     // Record the time the melody started
    tone(buzzerPin, melody[noteIndex], noteDuration[noteIndex]);

    // Prevent LCD interference by resetting its state
    lcdState = 0;                                // Ensure the LCD logic starts fresh
  }

  // Update the last button state
  lastButtonState = currentButtonState;

  // Non-blocking melody playback
  if (isPlayingMelody) {
    if (millis() - lastNoteTime >= noteDuration[noteIndex] * 1.3) {
      noTone(buzzerPin);                         // Stop the current tone
      noteIndex++;                               // Move to the next note
      if (noteIndex < 8) {                       // If there are more notes, play the next one
        tone(buzzerPin, melody[noteIndex], noteDuration[noteIndex]);
        lastNoteTime = millis();                 // Reset the timer for the next note
      } else {
        isPlayingMelody = false;                 // Stop the melody when all notes are played
      }
    }
  }
}

// Reset system to normal state
void return_normal() {
  servo.write(90);                      // Servo back to place
  digitalWrite(LedPin, LOW);            // Turn the LED off
  lcd.clear();                          // Clear the LCD

  // Stop the buzzer only if the melody is not playing
  if (!isPlayingMelody) {
    noTone(buzzerPin);                  // Stop the buzzer
  }
}

// Non-blocking light sensor handling
void lightSensor() {
  static unsigned long lastLightTime = 0; // Timer for light sensor updates
  if (millis() - lastLightTime >= 100) {  // Check every 100ms
    LdrValue = analogRead(LdrPin);        // Read the light sensor value
   // Serial.print("LDR = ");
    //Serial.println(LdrValue);            // Print the value for debugging
    if (LdrValue < 300)
      digitalWrite(LedPin, LOW);         // Turn off the LED if light is bright
    else
      digitalWrite(LedPin, HIGH);        // Turn on the LED if light is dim
    lastLightTime = millis();            // Reset the timer
  }
}

// Non-blocking LCD updates
void lcd1() {
  if (isPlayingMelody) return;          // Skip LCD updates while melody is playing

  if (lcdState == 0) {
    lcd.begin(16, 2);
    lcd.init();
    lcd.setBacklight(1);
    lcd.print("Welcome Oriya!");        // Display a welcome message
    lastLcdTime = millis();             // Start timing
    lcdState = 1;                       // Move to the next state
  } else if (lcdState == 1 && millis() - lastLcdTime >= 1000) {
    lcd.clear();                        // Clear the screen after 1 second
    lastLcdTime = millis();             // Start timing for backlight off
    lcdState = 2;                       // Move to the next state
  } else if (lcdState == 2 && millis() - lastLcdTime >= 1000) {
    lcd.setBacklight(0);                // Turn off the backlight
    lcdState = 0;                       // Reset state for next call
  }
}

// Servo movement logic
void servo1() {
  if (Direction) {
    angle = angle + 180;                // Move servo to 180 degrees
  } else {
    angle = angle - 180;                // Move servo back to 0 degrees
  }
  servo.write(angle);                   // Update the servo position
  delay(15);                            // Short delay for stability
  if (angle == 180) Direction = false;  // Reverse direction at max angle
  if (angle == 0) Direction = true;     // Reverse direction at min angle
}
