#include <Wire.h>
#include <LiquidCrystal_I2C_Hangul.h>
#include <Servo.h>

#define trigPin 9   // Trig pin connected to pin 9
#define echoPin 10  // Echo pin connected to pin 10
#define ledPin 13   // LED pin connected to pin 13
#define buttonPin 2 // Button pin connected to pin 2
#define servoPin 11 // Servo motor pin connected to pin 11
#define buzzerPin 4 // Buzzer pin connected to pin 4

long duration;
int distance;
Servo myServo;            // Create a servo object
LiquidCrystal_I2C_Hangul lcd(0x27, 16, 2);  // I2C address, 16 columns, 2 rows

int buttonState = 0;       // Variable to hold the button state
unsigned long lightTime = 0; // Timer for the light duration

// Variable to track whether the message has been displayed and the servo has been moved
bool isPersonDetected = false; // Flag to check if a person has been detected

// Define the melody notes and durations for "Marcia nuziale" by Mozart
int melody[] = {
  392, 392, 440, 440, 392, 392, 440, 440, 392, 392, 440, 440, 392, 392, 392, 440, // First phrase
  392, 392, 440, 440, 392, 392, 440, 440, 392, 392, 440, 440, 392, 392, 392       // Second phrase
};

int noteDurations[] = {
  400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, // 1st phrase duration
  400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400       // 2nd phrase duration
};


void setup() {
  Serial.begin(9600);      // Start serial communication at 9600 baud rate
  pinMode(trigPin, OUTPUT); // Set trig pin as output
  pinMode(echoPin, INPUT);  // Set echo pin as input
  pinMode(ledPin, OUTPUT);  // Set LED pin as output
  pinMode(buttonPin, INPUT); // Set button pin as input
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output

  myServo.attach(servoPin); // Attach the servo to pin 11
  lcd.begin(16, 2);         // Initialize the LCD with 16 columns and 2 rows
}

void loop() {
  // Send a 10 microsecond pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW);  

  // Measure the duration of the pulse from Echo pin
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (in cm)
  distance = duration * 0.0344 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // If the distance is less than or equal to 10 cm and person is not detected yet
  if (distance <= 10 && !isPersonDetected) {

    // Move the servo forward with a certain speed
    myServo.writeMicroseconds(1300);  // Forward
    Serial.println("Servo turns" );
    Serial.println("LED ON" );  
    digitalWrite(ledPin, HIGH);   // Turn on LED

    lcd.clear();
    lcd.print("Welcome!");         // Print "Welcome" message on the LCD
    delay(6000);  // Wait for 6 seconds before clearing the message
    lcd.clear();

    lcd.print("press button");
    delay(3000);
    // Set the flag that the person has been detected and the actions are performed
    isPersonDetected = true;
  } 
  // If the distance is greater than 10 cm, reset the detection flag
  else if (distance > 10) {
    // Reset the flag when the person is no longer detected (distance > 10 cm)
    isPersonDetected = false;
    digitalWrite(ledPin, LOW);    // Turn off LED
    lcd.clear();
    myServo.writeMicroseconds(1500);  // Stop the servo (stop the motor)
  }

  // Check the button state
  buttonState = digitalRead(buttonPin);
  
  // If the button is pressed, play a melody on the buzzer (without turning on the LED)
  if (buttonState == HIGH) {
    Serial.println("Button pressed");
    playMelody();  // Play the melody when the button is pressed
    lcd.clear();
    lcd.print("Enjoy :)");
    delay(5000);

  }

  delay(500);  // Wait for a while before the next measurement
}

// Function to play a melody on the buzzer
void playMelody() {
  Serial.println("Melody plays");
  for (int i = 0; i < 8; i++) {
    tone(buzzerPin, melody[i], noteDurations[i]); // Play the current note
    delay(noteDurations[i] * 1.3);  // Wait for the note duration before playing the next note
    noTone(buzzerPin); // Stop the tone after the note finishes
  }
}
