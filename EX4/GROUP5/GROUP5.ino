#include <PIDController.h>
#include <Encoder.h>

// Pins
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_CW 10
#define MOTOR_CCW 11
#define MOTOR_PWM 9

// PID Controller
PIDController pid;
Encoder myEnc(ENCODER_A, ENCODER_B);

// PID parameters
float Kp = 25, Ki = 0.5, Kd = 3000;
float goal_angle, current_angle, motor_pwm_value;
long initial_ticks; // נקודת האפס של האנקודר
bool is_first_iteration = true; // דגל לאיטרציה הראשונה
float last_stabilized_angle = 0; // זווית ההתייצבות הקודמת

// Function to convert encoder ticks to degrees
float tick_to_deg(long ticks) {
    return ticks * 360.0 / 440.0; // Assuming 440 ticks for a full rotation
}

// Motor control functions
void motor_forward(int pwm_value) {
    digitalWrite(MOTOR_CW, HIGH);
    digitalWrite(MOTOR_CCW, LOW);
    analogWrite(MOTOR_PWM, pwm_value);
}

void motor_reverse(int pwm_value) {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, HIGH);
    analogWrite(MOTOR_PWM, pwm_value);
}

void motor_resist() {
    analogWrite(MOTOR_PWM, 50); // Apply minimal power for resistance
}

// Function to initialize "zero position"
void set_zero_position() {
    initial_ticks = myEnc.read(); // Save the current encoder position as zero
}

// Function to get the desired angle from the user (first iteration only)
void get_user_angle() {
    if (is_first_iteration) {
        Serial.println("Enter target angle (degrees):");
        while (!Serial.available()); // Wait for user input
        goal_angle = Serial.parseFloat(); // Parse the input
        Serial.read();               // Clear the buffer
        pid.tune(Kp, Ki, Kd);        // Set PID parameters
        pid.setpoint(goal_angle);    // Set the target angle
        Serial.print("Target angle set to: ");
        Serial.println(goal_angle);
        is_first_iteration = false; // Mark first iteration as complete
    } else {
        pid.tune(Kp, Ki, Kd);        // Set PID parameters
        pid.setpoint(goal_angle);    // Keep using the same goal angle
    }
}

// Function to wait for pendulum movement with resistance
void wait_for_pendulum_movement() {
    Serial.println("You can now move the pendulum manually again.");
    bool moved = false;
    while (!moved) {
        long encoder_ticks = myEnc.read() - initial_ticks; // Calculate ticks relative to zero position
        current_angle = tick_to_deg(encoder_ticks);

        // Apply minimal resistance to motor
        motor_resist();

        // Check if the pendulum has moved by at least 45 degrees from the last stabilized angle
        if (abs(current_angle - last_stabilized_angle) >= 45.0) {
            moved = true;
        }

        delay(50); // Small delay to avoid excessive polling
    }
}
void motor_stop() {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
    analogWrite(MOTOR_PWM, 0);
}

// Function to measure and report the actual angle after stabilization
void measure_and_report_angle() {
    bool is_stabilized = false;
    while (!is_stabilized) {
        long encoder_ticks = myEnc.read() - initial_ticks; // Calculate ticks relative to zero position
        current_angle = tick_to_deg(encoder_ticks);

        if (abs(current_angle - goal_angle) < 5.0) { // If the error is small
            motor_stop();
            if (!is_stabilized) { // Print the message only once
                Serial.print("Pendulum stabilized at actual angle: ");
                Serial.println(current_angle); // Print the actual angle
                last_stabilized_angle = current_angle; // Update the last stabilized angle
                is_stabilized = true; // Mark as stabilized
            }
        } else {
            motor_pwm_value = pid.compute(current_angle);
            if (motor_pwm_value > 0) {
                motor_forward(motor_pwm_value);
            } else {
                motor_reverse(-motor_pwm_value);
            }
        }

        delay(50); // Small delay for stability
    }
}

// Setup function
void setup() {
    Serial.begin(9600);

    // Pin setup
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    // PID setup
    pid.begin();
    pid.limit(-255, 255); // Adjust PWM limits for motor power

    // Initialize zero position
    set_zero_position();
}

// Loop function
void loop() {
    // Get the target angle (only asks for user input on the first iteration)
    get_user_angle();

    // Wait for the user to move the pendulum by at least 45 degrees with resistance
    wait_for_pendulum_movement();

    // Wait for the pendulum to stabilize and report the angle
    measure_and_report_angle();
}