#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Wire.h>

// Pins
#define SERVO_PIN 9 
#define TRIG_PIN 5
#define ECHO_PIN 6 
#define BUTTON_PIN 7 
#define IR_PIN 11 

// Objects
Servo myServo; 
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Variables
int counter = 0;               // Counter for box openings
bool boxLocked = false;        // State to lock/unlock box
bool overrideMode = false;     // True if box stays open

// Remote control codes
const long IR_CODE_1 = 0x45; // code for activating override mode(=1)

// Initial settings
void setup() {
    Serial.begin(9600); 
    myServo.attach(SERVO_PIN); 
    pinMode(TRIG_PIN, OUTPUT); 
    pinMode(ECHO_PIN, INPUT);  
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
    IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK); 

    lcd.begin(16, 2); 
    lcd.backlight(); // Turn on LCD backlight
    displayCounter(); // Display the initial counter value
}

void loop() {
    // Automatic opening if an object is close
    if (!overrideMode && measureDistance() < 5) { // Trigger if distance is less than 5 cm
        openBox();
    }

    // Reset counter on button press
    if (digitalRead(BUTTON_PIN) == LOW) { 
        resetCounterAndClose();
        delay(500); 
    }

    // Receiving IR signal
    if (IrReceiver.decode()) { // Check if an IR signal is received
        handleIRInput(IrReceiver.decodedIRData.command); // Process the received command
        IrReceiver.resume(); // Prepare for the next signal
    }
}

// Function to measure distance using ultrasonic sensor
long measureDistance() {
    digitalWrite(TRIG_PIN, LOW); // Ensure the trigger pin is low
    delayMicroseconds(2); 
    digitalWrite(TRIG_PIN, HIGH); // Send a 10-microsecond pulse
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW); // Stop the pulse
    long duration = pulseIn(ECHO_PIN, HIGH); // Measure the time for the echo
    return (duration * 0.034) / 2; // Convert to distance in cm
}

// Display the opening counter on the LCD screen
void displayCounter() {
    lcd.clear(); // Clear the LCD screen
    lcd.setCursor(0, 0); 
    lcd.print("Open Count:"); 
    lcd.setCursor(0, 1); 
    lcd.print(counter); // Display the counter value
}

// Open the box
void openBox() {
    if (!boxLocked && !overrideMode) { // Proceed only if not locked or in override mode
        counter++; // Increment the counter
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Box Opened!"); // Display a message
        myServo.write(86); 
        delay(3100);          // Keep open for 3.1 seconds
        myServo.write(90); // Return to a neutral position
        displayCounter();     // Update the counter on the LCD
        delay(1000);
                          //  closing 
        myServo.write(100); 
        delay(2750); 
        myServo.write(90); 
        delay(1000);
    }
}

// Reset the counter and close the box
void resetCounterAndClose() {
    if (overrideMode) { // Reset only in override mode
        counter = 0; 
        overrideMode = false; // Disable override mode
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Counter Reset!");
        delay(2000); 
        myServo.write(100); // Additional closing action
        delay(2750);
        myServo.write(90); // Return to neutral position
        delay(1000);
        displayCounter(); // Update the counter on the LCD
    }
}

// Handle input from the remote control
void handleIRInput(unsigned long value) {
    if (!overrideMode) { // Check if not in override mode
        if (value == IR_CODE_1) { // Check if the received value matches "1"
            // Activate override mode
            overrideMode = true;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Maneger Mode!"); // Display override activation message
            lcd.setCursor(0, 1);
            lcd.print("Activated");
            Serial.println(F("Maneger mode activated")); 

            myServo.write(85); // Open box 
            delay(3100);          
            myServo.write(90); // Return to neutral position
            displayCounter();     // Update counter on the LCD
            delay(1000);
        } else {
            // Handle incorrect input
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Incorrect Code!"); // Display error message
            Serial.println(F("Incorrect code entered")); 
            delay(2000); 

            // Return to normal state
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Ready for Use"); // Display ready state
            lcd.setCursor(0, 1);
            lcd.print("Count: ");
            lcd.print(counter); // Display the current counter value
        }
    }
}
