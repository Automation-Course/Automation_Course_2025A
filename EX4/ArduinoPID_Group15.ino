#include <Encoder.h>
#include <PIDController.h>

//pins
#define counterClockWise 10
#define clockWise 11
#define enA 9 
int encoderPin2 = 2; 
int encoderPin3 = 3; 

// PID constants
float kp = 1; 
float ki = 0.17; 
float kd = 0.22; 
float targetAngle = 0.0; 
float setpoint; 
double angle = 0.0; // Current angle 


// Global variables
Encoder encoder(encoderPin2, encoderPin3); //reading encoder values
PIDController pidController; 

// Convert encoder ticks to degrees
float tick_to_deg(long tick) {
    return tick * 360.0 / 440.0; // normalized angle
}

// move the stick counterClockWise
void moveForward() { 
  digitalWrite(counterClockWise, HIGH);
  digitalWrite(clockWise, LOW);
}

// move the stick clockWise
void moveReverse() {
  digitalWrite(counterClockWise, LOW);
  digitalWrite(clockWise, HIGH);
}

void getUserInput() { // Get user's input 
  if (Serial.available() > 0) {
    float Input = Serial.parseFloat();
    Serial.read();   
    setpoint = Input; // Assign the user input to the setpoint 
    pidController.setpoint(setpoint);
  }
}

// Setup function
void setup() {
  // Initialize motor pins as output
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(counterClockWise, OUTPUT);
  pinMode(clockWise, OUTPUT);
  pidController.begin(); // Initialize the PID controller
  pidController.limit(-180, 180); // Set the output limits of the PID controller
  pidController.tune(kp, ki, kd); // Set PID parameters
  pidController.setpoint(targetAngle); // Set the target angle for the PID controller
}

// Loop function
void loop() {

  // Read encoder and calculate current angle 
  long encoderCount = encoder.read();
  angle = tick_to_deg(encoderCount);
  getUserInput();
  // Calculates control signal and the difference of angle between the current angle and the target angle
  double control = pidController.compute(angle);

  Serial.print("angle: ");
  Serial.println(angle);// prints the current angle
  
  if (control > 0.0) {
    moveForward();
  } else {
    moveReverse();
  }
  analogWrite(enA, abs(control * 10.2));
}