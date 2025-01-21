#include <Encoder.h> // Encoder defining
#include <PIDController.h>  //PID Defining
#define ENCODER_A 2
#define ENCODER_B 3   // Encoder output to Arduino Interrupt pin ncoderPinB
#define MOTOR_CCW 11  // Motor defining
#define MOTOR_CW 10
#define enA 9
float motor_pwm_value = 0;
float desired_angle = 0;
float current_deg = 0;
float encoder_count = 0;
float tick_to_deg1 = 0;
float Motor_output = 0;
float input_to_PID = 0;
//K's Parameters
float Kp = 3;
float Ki = 0.15;
float Kd = 9;
PIDController pid;  // Create an instance of the PID controller class, called "pid"
Encoder myEnc(ENCODER_A, ENCODER_B);


//functions
float get_degree() {
  encoder_count = myEnc.read();
  tick_to_deg1 = tick_to_deg(encoder_count);
  return tick_to_deg1;
}
float tick_to_deg(long tick) {
  return tick * 360.0 / 440.0;
}
void forward() {
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
}
void reverse() {
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, HIGH);
}
/**setup ****/
void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);
  pid.begin();                  // initialize the PID instance
  pid.limit(-180, 180);         // Limit the PID output between 0 and 255, this is
  pid.tune(Kp, Ki, Kd);         // Tune the PID, arguments: kP, kI, kD
  pid.setpoint(desired_angle);  // The "goal" the PID controller tries to "reach"
  Serial.print("Enter the degree between -15 to +15 degrees:");
}
void loop() {
  if (Serial.available() > 0) {  // Wait for User to Input Data
    desired_angle = Serial.parseFloat();
    if (desired_angle > 45) {  //max input angle we allow is +-45
      desired_angle = 45;
    }
    if (desired_angle < -45) {
      desired_angle = -45;
    }
    Serial.read();  // get the ENTER char from the serial
    pid.setpoint(desired_angle);
  }
  current_deg = get_degree();
  Serial.println(current_deg);
  motor_pwm_value = pid.compute(current_deg);  //compute the PWM value for the motor based on current angle
  Motor_output = abs(255 * motor_pwm_value * 4 / 100);
  if (motor_pwm_value > 0.0) {
    forward();
  } else {
    reverse();
  }
  analogWrite(enA, Motor_output);  // send PWM value to motor
}
