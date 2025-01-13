#define ENCODER_A 2  // Encoder output to Arduino pin A
#define ENCODER_B 3  // Encoder output to Arduino pin B
#define MOTOR_CCW 11 // Motor counter-clockwise control pin
#define MOTOR_CW 10  // Motor clockwise control pin
#define enA 9        // Motor PWM control pin

#include <Encoder.h>
#include <PIDController.h>

// Variables and constants
float encoder_count = 0;   // Stores the current encoder count
float tick_to_deg1 = 0;    // Tick-to-degree conversion factor
int motor_pwm_value = 0;   // PWM value for motor speed control
float userInput = 0;       // User input from the serial monitor

// PID controller object
PIDController pid;

// Encoder object
Encoder myEnc(ENCODER_B, ENCODER_A);

/** Functions ***/
void forward() {
  // Move motor forward
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
}

void reverse() {
  // Move motor backward
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, HIGH);
}

void get_user_input() {
  // Get user input from the serial monitor
  if (Serial.available() > 0) {
    userInput = Serial.parseFloat();
    Serial.read(); // Clear the buffer (e.g., ENTER key)
    Serial.println(userInput);
  }
}

float tick_to_deg(long tick) {
  // Convert encoder ticks to degrees
  return tick * 360.0 / 440.0;
}

/*** Setup ***/
void setup() {
  Serial.begin(9600);             // Start serial communication
  pinMode(enA, OUTPUT);           // Set motor PWM pin as output
  pinMode(MOTOR_CW, OUTPUT);      // Set motor clockwise control pin
  pinMode(MOTOR_CCW, OUTPUT);     // Set motor counter-clockwise control pin

  pid.begin();                    // Initialize the PID controller
  pid.limit(-255, 255);           // Limit PID output between -255 and 255
  pid.tune(8.7, 2, 0.62);         // Set PID parameters

  Serial.print("Write the desired angle: ");
}

/*** Loop ***/
void loop() {
  if (Serial.available() > 0) { // Wait for user to input data
    get_user_input();
  }

  encoder_count = myEnc.read();              // Read encoder count
  tick_to_deg1 = tick_to_deg(encoder_count); // Convert encoder count to degrees

  // Normalize angle to be within 0-360 degrees
  while (abs(tick_to_deg1) > 360) {
    if (tick_to_deg1 < 0) {
      tick_to_deg1 += 360;
    } else {
      tick_to_deg1 -= 360;
    }
  }

  // Compute the PWM value for the motor based on the current angle
  motor_pwm_value = pid.compute(userInput - tick_to_deg1);

  // Determine motor direction
  if (tick_to_deg1 < userInput) {
    reverse();
  } else {
    forward();
  }

  // Send PWM value to motor
  analogWrite(enA, abs(motor_pwm_value));

  delay(1); // Short delay
}
