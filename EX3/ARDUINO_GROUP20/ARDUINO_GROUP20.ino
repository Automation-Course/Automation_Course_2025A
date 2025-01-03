
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define led 12          // Pin for the LED
#define button 7        // Pin for the button
#define DHTPIN 2        // Pin for the temperature sensor
#define DHTTYPE DHT11   // Define the type of sensor (DHT11)
#define servoPin 9      // Pin for the Servo motor
#define buzzerPin 13    // Pin for the buzzer

Servo servo;

// Create a DHT object
DHT dht(DHTPIN, DHTTYPE);

// Create an LCD object. Set the I2C address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool systemActive = false;   // System state (OFF by default)
bool lastButtonState = LOW; // Track the previous button state (default HIGH because of INPUT_PULLUP)
int angle = 0;               // Current servo angle
bool direction = true;       // Direction of servo movement

void setup() {
  // Initialize the LED, button, and buzzer
  pinMode(led, OUTPUT);
  pinMode(button, INPUT_PULLUP); // Use internal pull-up resistor
  //pinMode(buzzerPin, OUTPUT);    // Set the buzzer pin as output
  digitalWrite(led, LOW);        // Ensure the LED starts OFF
  digitalWrite(buzzerPin, LOW);  // Ensure the buzzer starts OFF

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the Serial Monitor
  Serial.begin(9600);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.print("System Ready"); // Display initial message
  delay(2000);              // Wait 2 seconds
  lcd.clear();

  // Attach the servo and set initial position
  //servo.attach(servoPin);
  //servo.write(0); // Ensure the servo starts at 0 degrees (inactive)
}

void loop() {
  int buttonState = digitalRead(button); // Read the current state of the button

  // Check for a button press (transition from HIGH to LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    systemActive = !systemActive;   // Toggle the system state

    lcd.clear();
    if (systemActive) {
      Serial.println("System On"); 
      lcd.print("System On");
      digitalWrite(led, HIGH);      // Turn the LED on
      buzzOn(200);                  // Emit a single buzz for 200ms
    } else {
      Serial.println("System Off"); 
      lcd.print("System Off");
      digitalWrite(led, LOW);       // Turn the LED off
      servo.write(0);               // Reset the servo to 0 degrees
      digitalWrite(buzzerPin, LOW); // Ensure the buzzer is OFF
    }

    delay(200); // Debounce delay
  }

  // If the system is active, control the servo and display temperature
  if (systemActive) {
    lcd.clear();
    float temp = dht.readTemperature(); // Read temperature in Celsius
    servo.attach(servoPin);
    Serial.println("servo on"); // Reattaches the servo
    pinMode(buzzerPin, OUTPUT);    // Set the buzzer pin as output
    
    if (isnan(temp)) {
      lcd.clear();
      lcd.print("Sensor Error");
      return; // Skip the rest of the loop if sensor reading fails
    }

    if (temp < 24) {
      lcd.clear();
      lcd.print("Temp Low");
      systemActive = false;
    } else {
      // Move the servo
      if (direction) {
        angle++;
      } else {
        angle--;
      }
      servo.write(angle);
      delay(15); // Smooth movement
      if (angle == 180) direction = false;
      if (angle == 0) direction = true;

      // Display the temperature
      lcd.clear();
      lcd.print("Temp: ");
      lcd.print(temp, 2); // Print temperature with 2 decimal places
      lcd.print(" C");
      Serial.print("Temp:"); 
      Serial.print(temp);
    }
  }

  // If the system is not active, ensure all components are off
  if (!systemActive) {
    lcd.clear();
    servo.detach(); // Stops servo  
    pinMode(buzzerPin,INPUT); // Stops buzzer  
    digitalWrite(buzzerPin, LOW); // Ensure the buzzer stays off
    digitalWrite(led, LOW);       // Ensure the LED stays off
   Serial.println("buzzer off"); 
   Serial.println("servo off"); 


  }

  // Update the last button state
  lastButtonState = buttonState;
}

// Function to activate the buzzer for a specified duration
void buzzOn(int duration) {
  digitalWrite(buzzerPin, HIGH); // Turn the buzzer on
  delay(duration);               // Wait for the specified duration
  digitalWrite(buzzerPin, LOW);  // Turn the buzzer off
  Serial.println("buzzer on"); 

}
