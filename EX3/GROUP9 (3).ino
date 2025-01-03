#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Initialize the LCD with I2C address 0x27 (common for many I2C LCDs)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize Servo
Servo MG995_Servo;

// Pins
#define PIR_PIN 3         // Pin for PIR sensor (motion detection)
#define BUZZER_PIN 8      // Pin for Buzzer (active buzzer connected to pin 8)
#define BUTTON1_PIN 2     // Pin for Button 1
#define BUTTON2_PIN 4     // Pin for Button 2
#define BUTTON3_PIN 5     // Pin for Button 3
#define SERVO_PIN 6       // Pin for Servo motor

// Global variables
int peopleCount = 0;           // Counter for the number of people detected
bool happyHourActive = false;  // Flag to indicate if Happy Hour is active
bool happyHourCompleted = false; // Flag to ensure Happy Hour only runs once
int pirState = 0;              // Variable to store the state of the PIR sensor

void setup() {
  pinMode(PIR_PIN, INPUT);         // Set PIR_PIN as INPUT to read sensor data
  pinMode(BUZZER_PIN, OUTPUT);     // Set BUZZER_PIN as OUTPUT to control buzzer
  digitalWrite(BUZZER_PIN, HIGH); // Make sure buzzer is off initially (HIGH turns off active buzzer)
  
  pinMode(BUTTON1_PIN, INPUT_PULLUP); // Set Button pins as input with pull-up resistors
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);

  // Initialize the LCD display
  lcd.init();          // Initialize the LCD
  lcd.backlight();     // Turn on the backlight of the LCD
  lcd.print("System Ready"); // Display "System Ready"
  delay(2000);         // Wait for 2 seconds
  lcd.clear();         // Clear the LCD display

  // Start Serial Monitor for debugging
  Serial.begin(9600);

  // Initialize the Servo motor
  MG995_Servo.attach(SERVO_PIN); // Attach the Servo to the defined pin
}




void loop() {
  // If Happy Hour has already been completed, exit the loop to prevent re-running it
  if (happyHourCompleted) {
    return;  // Exit the loop, do nothing further
  }

  pirState = digitalRead(PIR_PIN);  // Read the PIR sensor (HIGH means motion detected)

  // Print PIR sensor state to Serial Monitor for debugging
Serial.println("PIR State: ");  
Serial.println(pirState);

  // If movement is detected, increment people count and update LCD display
  if (pirState == HIGH) {
    peopleCount++;  // Increase the number of people detected
    Serial.println ("People Count Updated: ");
    Serial.println(peopleCount);  // Print current people count to Serial Monitor
    lcd.clear();  // Clear the LCD
    lcd.print("People: ");  // Display label "People:"
    lcd.print(peopleCount); // Display the current people count
    delay(2000);  // Wait for 2 seconds to avoid multiple detections from the same motion

    // If there are more than 10 people, activate Happy Hour
    if (peopleCount > 10 && !happyHourActive) {
      Serial.println ("Activating Happy Hour...");
      activateHappyHour();  // Call the function to activate Happy Hour
    }
  } else {
    lcd.clear();  // Clear the LCD
    lcd.print("People: ");  // Display label "People:"
    lcd.print(peopleCount); // Display the current people count
  }

  rotate(90);  // Call rotate function to move the servo (separate logic)
}

void activateHappyHour() {
  // Set Happy Hour flag to true to prevent repeated activation
  happyHourActive = true;
  Serial.println("Happy Hour Activated!");
  lcd.clear();  // Clear the LCD
  lcd.print("Happy Hour!");  // Display "Happy Hour" message

  // Start buzzer sound for 1 second to indicate Happy Hour has started
  digitalWrite(BUZZER_PIN, LOW);  // Turn on the buzzer (signal LOW for active buzzer)
  Serial.println ("Buzzer On");
  delay(1000);  // Wait for 1 second (buzzer will sound for this period)
   Serial.println ("setting servo speed");
  digitalWrite(BUZZER_PIN, HIGH);  // Turn off the buzzer after 1 second
  Serial.println ("Buzzer Off");

 
  // Display the options for the user to choose a drink
  lcd.clear();  // Clear the LCD
  lcd.print("Choose Drink:");  // Display "Choose Drink:"
  lcd.setCursor(0, 1);  // Set cursor to second line
  lcd.print("1 2 3");    // Display the options "1 2 3"

  // Move the servo to indicate the selection process
  MG995_Servo.write(90);  // Move the servo to 90 degrees
  delay(1000);  // Wait for 1 second
  MG995_Servo.write(0);   // Move the servo back to 0 degrees
  delay(1000);  // Wait for 1 second

  // Wait for user selection (via buttons)
  bool choiceMade = false;
  while (!choiceMade) {
    if (digitalRead(BUTTON1_PIN) == LOW) {  // If Button 1 is pressed
    Serial.println("Drink 1 Selected");
      lcd.clear();
      lcd.print("Drink 1 choosen!");  // Display promo for Drink 1
       Serial.println ("stopping servo");
      choiceMade = true;  // Mark that a choice was made
    } else if (digitalRead(BUTTON2_PIN) == LOW) {  // If Button 2 is pressed
     Serial.println("Drink 2 Selected");
      lcd.clear();
      lcd.print("Drink 2 choosen!");  // Display promo for Drink 2
       Serial.println ("stopping servo");
      choiceMade = true;  // Mark that a choice was made
    } else if (digitalRead(BUTTON3_PIN) == LOW) {  // If Button 3 is pressed
    Serial.println("Drink 3 Selected");
      lcd.clear();
      lcd.print("Drink 3 choosen!");  // Display promo for Drink 3
       Serial.println ("stopping servo");
      choiceMade = true;  // Mark that a choice was made
    }
    delay(200);  // Debounce delay to avoid multiple button presses being registered
  }

  // Mark Happy Hour as completed to prevent re-running it
  happyHourCompleted = true;  // Flag to prevent re-running Happy Hour

  // Reset system after selection (no more activation)
  resetSystem();  // Call reset function to reset the system
}

void rotate(int speed) {
  
  MG995_Servo.write(speed);  // Move the servo to the specified speed/angle
  delay(900);  // Keep the servo rotating for a short time (900 ms)
  MG995_Servo.write(90);  // Return the servo to the neutral position (90 degrees)
}   

void resetSystem() {
  // Reset the system, but do not allow Happy Hour to run again
  happyHourActive = false;  // Reset Happy Hour flag to prevent re-entry
  peopleCount = 0;  // Reset people count
  delay(3000);  // Wait for 3 seconds
  lcd.clear();  // Clear the LCD display
  lcd.print("System Ready");  // Display "System Ready"
  delay(2000);  // Wait for 2 seconds
  lcd.clear();  // Clear the LCD display
}
