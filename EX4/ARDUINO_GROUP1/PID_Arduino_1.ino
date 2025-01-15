#include <Encoder.h>
#include <PIDController.h>

#define MOTOR_DIRECTION_CCW 11
#define MOTOR_DIRECTION_CW 10
#define MOTOR_PWM_PIN 9
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// Global variables
PIDController pidController;
Encoder pendulumEncoder(ENCODER_PIN_B, ENCODER_PIN_A);

int currentEncoderTicks = 0;          
float currentAngleInDegrees = 0.0;    
int motorPwmValue = 0;                
float proportionalGain = 13.0, integralGain = 1.1, derivativeGain = 3.5; 
float targetAngle = 0.0;              
bool isNewTargetSet = false;          

// Function to convert encoder ticks to degrees
float convertTicksToDegrees(long encoderTicks) {
    return encoderTicks * 360.0 / 440.0; // Convert encoder ticks to degrees based on your encoder's resolution
}

// Function to rotate the motor clockwise
void rotateMotorClockwise() {
    digitalWrite(MOTOR_DIRECTION_CW, HIGH);
    digitalWrite(MOTOR_DIRECTION_CCW, LOW);
}

// Function to rotate the motor counter-clockwise
void rotateMotorCounterClockwise() {
    digitalWrite(MOTOR_DIRECTION_CW, LOW);
    digitalWrite(MOTOR_DIRECTION_CCW, HIGH);
}

// Function to get the desired angle from the user via serial input
void setTargetAngle() {
    targetAngle = Serial.parseFloat();    // Read the target angle from user input
    Serial.read();                         // Clear the buffer
    if (targetAngle < -15 || targetAngle > 15) {
        Serial.println("Error: Enter a degree between -15 and 15.");
    } else {
        pidController.setpoint(targetAngle);  
        isNewTargetSet = true;                
        Serial.print("New target set: ");
        Serial.println(targetAngle);
    }
}

void setup() {
    Serial.begin(9600);                   
    pinMode(MOTOR_PWM_PIN, OUTPUT);        
    pinMode(MOTOR_DIRECTION_CW, OUTPUT);  
    pinMode(MOTOR_DIRECTION_CCW, OUTPUT);

    // Initial motor direction set to clockwise
    rotateMotorClockwise();

    // Initialize PID controller
    pidController.begin();
    pidController.limit(-255, 255);                // Limit the PID output to a suitable range for PWM
    pidController.tune(proportionalGain, integralGain, derivativeGain); // Set PID constants
    pidController.setpoint(targetAngle);          // Set the initial target angle for the PID controller

    Serial.println("Enter the target angle (-15 to 15):"); // Prompt the user for input
}

void loop() {
    // Check if there is any serial input (user provides a new target angle)
    if (Serial.available() > 0) {
        setTargetAngle();   // Get the new target angle from the user
    }

    // Read the current value from the encoder
    currentEncoderTicks = pendulumEncoder.read();
    currentAngleInDegrees = convertTicksToDegrees(currentEncoderTicks);  // Convert encoder ticks to angle in degrees

    // Keep the angle within the range of -360 to 360 degrees
    while (abs(currentAngleInDegrees) > 360) {
        if (currentAngleInDegrees < 0) {
            currentAngleInDegrees += 360;
        } else {
            currentAngleInDegrees -= 360;
        }
    }

    // Compute the new PWM value based on the current angle using the PID controller
    motorPwmValue = float(pidController.compute(currentAngleInDegrees));

    // Determine the direction of rotation based on the PID output (motorPwmValue)
    if (motorPwmValue < 0) {
        rotateMotorCounterClockwise();   // Rotate motor counter-clockwise if motorPwmValue is negative
    } else {
        rotateMotorClockwise();   // Rotate motor clockwise if motorPwmValue is positive
    }

    // Send the PWM value to the motor
    analogWrite(MOTOR_PWM_PIN, abs(motorPwmValue));   // Apply PWM signal to motor

    // Update the serial monitor with the current angle and target angle if a new target was set
    if (isNewTargetSet) {
        Serial.print("Current Angle: ");
        Serial.println(currentAngleInDegrees);  // Print the current angle
        Serial.print("Target Angle: ");
        Serial.println(targetAngle);            // Print the target angle
        isNewTargetSet = false;                 // Reset the flag after displaying the data
    }

    delay(10);  // Small delay for stability
}