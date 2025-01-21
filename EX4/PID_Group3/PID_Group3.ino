#include <PID_v1.h>

// Motor control pins
const int ENA = 9;  // PWM speed control
const int IN1 = 10; // Direction control 1
const int IN2 = 11; // Direction control 2

// Encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Variables for encoder position tracking
volatile long encoderPos = 0;
const float STEPS_PER_REVOLUTION = 220.0; // Encoder steps per revolution
const float DEGREES_PER_STEP = 360.0 / STEPS_PER_REVOLUTION;

// PID variables
double Setpoint = 0;  // Target angle
double Input = 0;     // Current angle
double Output = 0;    // Output for motor control

// PID parameters
double Kp = 3, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Timing variables
bool initialPhase = true; // Indicates if the system is in the initial phase
unsigned long phaseStartTime;
const unsigned long initialPhaseDuration = 5000; // Duration of the initial phase in milliseconds

void setup() {
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // PWM limits
  myPID.SetSampleTime(10);         // PID calculation interval

  // Start initial phase
  phaseStartTime = millis();
  Serial.println("Starting oscillation phase...");
  Serial.println("After oscillation, enter a target angle within ±15° of the current angle.");
}

void loop() {
  // Update current angle
  Input = encoderPos * DEGREES_PER_STEP;

  if (initialPhase) {


    // Check if initial phase duration is over
    if (millis() - phaseStartTime >= initialPhaseDuration) {
      initialPhase = false;
      Setpoint = 0; // Reset target angle to center
      Serial.println("Oscillation phase complete. Enter a target angle within ±15°:");
    }
  } else {
    // Stabilization phase: Accept user input for target angle
    if (Serial.available() > 0) {
      processUserInput();
    }
  }

  // Compute PID output
  myPID.Compute();

  // Control motor using the PID output
  controlMotor(Output);

  // Print current system state for analysis every 100ms
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= 100) {
    Serial.print("Angle: ");
    Serial.print(Input);
    Serial.print(" Target: ");
    Serial.print(Setpoint);
    Serial.print(" Output: ");
    Serial.println(Output);
    lastPrintTime = currentTime;
    
  }
}

//---functions---

void processUserInput() {
  String input = Serial.readStringUntil('\n');
  double newTarget = input.toFloat();

  // Validate target angle
  if (abs(newTarget - Input) <= 15.0) {
    Setpoint = newTarget; // Update target angle
    Serial.print("New target angle set to: ");
    Serial.println(Setpoint);
  } else {
    Serial.println("Error: Target angle out of range. Enter a value within ±15° of the current angle.");
  }
}

void controlMotor(double pidOutput) {
  // Stop motor if within 2° of target
  if (abs(Input - Setpoint) < 1.0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    return;
  }

  // Determine motor direction and apply speed
  if (pidOutput > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Apply PWM speed with constraints
  analogWrite(ENA, constrain(abs(pidOutput),80, 255));
}

void encoderISR() {
  // Update encoder position
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);

  if (a == b) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}
