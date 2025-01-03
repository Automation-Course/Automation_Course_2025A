#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin Definitions
#define TRIG_PIN 13      // Trigger pin for the ultrasonic sensor
#define ECHO_PIN 12      // Echo pin for the ultrasonic sensor
#define GREEN_LED 4      
#define RED_LED 2        
#define BUTTON 7         
#define SERVO_PIN 11     // Servo motor pin
#define OneWireBus A0    // Temperature sensor pin 

OneWire onWire(OneWireBus); // Initialize OneWire for the Dallas temperature sensor
DallasTemperature sensors(&onWire); // Initialize Dallas temperature library

// Variables to track system state
int buttonState = 0;        // Tracks the state of the button (pressed or not)
float temperature;          // Stores the temperature value
//long duration;              // Duration for the ultrasonic sensor
int distance;               // Calculated distance from ultrasonic sensor
bool light = false;         // Tracks the LED state
int counter = 0;            // Counter for button presses
bool open = false;          // Tracks if the servo door is open
LiquidCrystal_I2C lcd_1(0x27, 16, 2); // LCD 
Servo servoMotor;           // Servo motor object
int deg = 90;               // Servo angle


void setup() {
  lcd_1.begin(16, 2);         // Initialize the LCD
  lcd_1.backlight();          
  pinMode(TRIG_PIN, OUTPUT);  
  pinMode(ECHO_PIN, INPUT);   
  servoMotor.attach(SERVO_PIN); // Attach servo to pin 11
  pinMode(GREEN_LED, OUTPUT); 
  pinMode(RED_LED, OUTPUT);   
  pinMode(BUTTON, INPUT);     
  Serial.begin(9600);        
  sensors.begin();           
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
}

void loop() {
 
  distance = getDistance(); //Get distance from sensor
  buttonState = digitalRead(BUTTON); 

  
  if (buttonState == HIGH) { 
    Serial.println("The button was pressed, the system was activated ");
    counter += 1; // Increment the counter
    if (counter % 2 != 0) { // Odd count: turn on the system
      digitalWrite(GREEN_LED, HIGH);
      Serial.println("Green led is on");
      digitalWrite(RED_LED, LOW);
      light = true; // System active
      lcd_1.print("Or's restaurant!"); 
      delay(2000);
      lcd_1.clear();
     
      if (!open) {
        OpenDoor(); // Open the door if it's not already open
        open = true;
      }
    } else { // Even count: turn off the system
      Serial.println("The button was pressed, the system is turned off ");
      digitalWrite(GREEN_LED, LOW);
      Serial.println("Green led is off");
      digitalWrite(RED_LED, LOW);
      Serial.println("Red led is off");
      light = false; // System inactive
      if (open) {
        CloseDoor(); // Close the door if it's open
        open = false;
      }
      lcd_1.clear();
    }
    
  }
  
  // Check distance and light state
  if (distance < 12 && light) { // If plate is detected and system is active
    lcd_1.clear();
    lcd_1.print("Plate Detected"); // Display detection message
    Serial.println("Plate Detected");
    delay(500);
    lcd_1.clear();
    
    sensors.requestTemperatures(); // Request temperature reading
    temperature = sensors.getTempCByIndex(0); // Get temperature value
    
    if (temperature >= 30) { // If plate is too hot
      digitalWrite(RED_LED, HIGH); // Turn on red LED
      Serial.println("Red led is on");
     // isServoOpen = false; // Mark servo as closed
      lcd_1.clear();
      if (open) {
        CloseDoor(); // Close the door
        open = false;
      }
      lcd_1.print("Hot plate"); // Display warning
      Serial.println("The plate is too hot");
      delay(500);
      lcd_1.clear();
    } else if (temperature < 30) { // If plate temperature is normal
     
      digitalWrite(RED_LED, LOW); // Turn off red LED
      lcd_1.clear();
      lcd_1.print("Take your plate"); // Display message
      Serial.println("Take your plate, temperature is normal");
      delay(500);
      lcd_1.clear();
      if (!open) {
        OpenDoor(); // Open the door
        open = true;
      }
    }
  }
}

// Function to open the door using the servo
void OpenDoor() {
  deg = 180; // Set servo angle to open
  servoMotor.write(deg);
  Serial.println("The door is open");
  delay(900);
  deg = 90; // Reset servo angle to default
  servoMotor.write(deg);
}

// Function to close the door using the servo
void CloseDoor() {
  deg = 0; // Set servo angle to close
  servoMotor.write(deg);
  Serial.println("The door is closed");
  delay(900);
  deg = 90; // Reset servo angle to default
  servoMotor.write(deg);
}

// Function to calculate distance using the ultrasonic sensor
int getDistance() {
  long duration; // Time taken for echo
  int distance;  // Calculated distance

  // Send trigger signal to ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure duration of echo signal
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  distance = duration * 0.034 / 2;
  return distance;
}