#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

//  Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

//  Initialize Servo
Servo servo;

// IR Remote
#define IR_RECEIVE_PIN 2  // Pin for connecting IR sensor
unsigned long authorizedIRCode = 0x1C; // Specific button code for approval
unsigned long denyOrderIRCode = 0x16; // Specific button code for denying an order
bool irAuthorized = false;

// Pin definitions
const int addOrderButton = 6;  // Button for adding an order
const int ServeOrderButton = 8;      // Chef approval button
const int greenLed = 7;             // Green LED indicator
const int redLed = 4;          // Red LED indicator
const int servoPin = 5;           // Servo motor pin

int pendingOrders = 0;

// Debounce variables
unsigned long lastDebounceTimeOrder = 0;
unsigned long lastDebounceTimeServe = 0;
const unsigned long debounceDelay = 200;

void setup() {
  //  Initialize pins 
  pinMode(addOrderButton, INPUT_PULLUP); // Button for adding an order
  pinMode(ServeOrderButton, INPUT_PULLUP);     // Serve order button
  pinMode(greenLed, OUTPUT);                 // Green LED indicator
  pinMode(redLed, OUTPUT);              // Red LED indicator

  //  Attach servo 
  servo.attach(servoPin);
  servo.write(90); // Start at stop position

  //  Initialize LCD
  lcd.init();
  lcd.backlight();
  
  //  Display Welcome Message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Ben-Gurion Rest.");
  delay(5000); // Display for 5 seconds

  activateLCD(); // Switch to the pending orders display

  //  Initialize IR Remote
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Activate IR receiver
  Serial.println("Use buttons and IR remote");
}

void loop() {
  //  **Button for adding an order**
  if (digitalRead(addOrderButton) == LOW && millis() - lastDebounceTimeOrder > debounceDelay) {
    pendingOrders++;
    Serial.println("Order Added!");
    activateLCD();
    updateLCD();
    lastDebounceTimeOrder = millis();
  }

  // **Approval or denial with IR remote button**
  if (IrReceiver.decode()) {
    Serial.print("IR Code: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX); // Display button code

    // ✔️ **Approve order with IR**
    if (IrReceiver.decodedIRData.command == authorizedIRCode && IrReceiver.decodedIRData.command != 0x0) {
      irAuthorized = true;
      Serial.println("Chef aprroved");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Chef aprroved ");
      delay(2000); // Short display delay
      activateLCD();
    }
    //  **Deny order with IR**
    else if (IrReceiver.decodedIRData.command == denyOrderIRCode && IrReceiver.decodedIRData.command != 0x0) {
      Serial.println("Chef Denied");
      denyOrder(); // Call function to deny order
    }
    // **Unrecognized button**
    else {
      Serial.println("Eror buttom");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Eror buttom");
      delay(2000);
      activateLCD();
    }

    IrReceiver.resume(); // Reset IR sensor for new input
  }

  //  **Chef approval button**
  if (digitalRead(ServeOrderButton) == LOW && irAuthorized && pendingOrders > 0 && millis() - lastDebounceTimeServe > debounceDelay) {
    pendingOrders--;
    Serial.println("The order is ready");
    activateLCD();
    updateLCD();
    operateServo(); // Operate continuous servo
    irAuthorized = false; // Reset authorization
    lastDebounceTimeServe = millis();
  }
}

// Update LCD with pending orders count
void updateLCD() {
  lcd.setCursor(0, 1);
  lcd.print("                "); // Clear row
  lcd.setCursor(0, 1);
  lcd.print(pendingOrders);
}

//  Activate LCD to display status**
void activateLCD() {
  lcd.display();
  lcd.setCursor(0, 0);
  lcd.print("Pending Orders:");
  updateLCD();
}

//  Operate continuous servo with LED on
void operateServo() {
  Serial.println("The order is on the way");
  digitalWrite(greenLed, HIGH); // Green LED turns on
  servo.write(0); // Move forward at full speed
  delay(5000); // 5 seconds delay

  Serial.println("Stopping Servo");
  servo.write(90); // Stop servo
  delay(500); // Short delay

  digitalWrite(greenLed, LOW); // Green LED turns off
}

// Function to deny an order via IR**
void denyOrder() {
  Serial.println(" Order Denied!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Order Denied!");
  digitalWrite(redLed, HIGH); // Red LED turns on
  delay(5000); // 5 seconds delay
  digitalWrite(redLed, LOW); // Red LED turns off
  activateLCD();
}

