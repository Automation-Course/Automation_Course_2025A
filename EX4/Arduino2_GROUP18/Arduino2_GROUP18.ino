#define MOTOR_CCW 11
#define MOTOR_CW 10
#define enA 9
#define ENCODER_A 2 
#define ENCODER_B 3 // Encoder output to Arduino Interrupt pin encoderPinB
#include <Encoder.h>
int prev_deg=0;
bool flag=true;
int motor_pwm_value;
int Power_In_Percents;
int printdeg=0;
int encoder_count = 0; // stores the current encoder count
float tick_to_deg1 ;
int user_deg=1000;
#include <PIDController.h>    
PIDController pid;
Encoder myEnc(ENCODER_B, ENCODER_A);

/*******functions******/
void forward(){//clockwise
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
  }

void reverse(){//counter clockwise
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, HIGH);
  }
 

float tick_to_deg(long tick){//calculates degree
    return tick*360.0/440.0;
  }

/*******setup *******/
void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);

  // Set initial rotation direction
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
  pid.begin(); // Initialize the PID controller
  pid.limit(-255, 255); // The PID output can be limited to values between -255 to 255
  pid.tune(32.5, 0.05,2000 ); // Set PID parameters
  pid.setpoint(user_deg); // the goal angle

  }
void loop() {
  
 
  if (flag)
  {
    if(Serial.available()>0)//until user input 
      user_deg = Serial.parseInt();
    if(user_deg<=360)
      flag=false;
    if(user_deg<0)
      user_deg = 360 + user_deg;
    pid.setpoint(user_deg);
  }
  else
  {
   // calculate motor direction
  if (motor_pwm_value <0 )
    reverse();
  else 
   forward();
  
  // send PWM value to motor
  analogWrite(enA, abs(motor_pwm_value));
 
 delay(1);
 encoder_count = myEnc.read();//reads encoder
 
  tick_to_deg1 = tick_to_deg(encoder_count);//
  if (abs(tick_to_deg1) > 360)
	tick_to_deg1 = int(abs(tick_to_deg1)) % 360;//calculate degree
  printdeg=tick_to_deg1;
  if(tick_to_deg1<0)
    printdeg=tick_to_deg1+360;
  if(prev_deg!=tick_to_deg1)
  {
   Serial.print("tick_to_deg1 = ");
   Serial.println(printdeg);
   prev_deg=tick_to_deg1;
  }
  motor_pwm_value = float(pid.compute(tick_to_deg1));  //compute the PWM value for 					//the motor based on current angle


  
  }
}