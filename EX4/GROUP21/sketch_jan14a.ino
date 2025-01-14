#include <PIDController.h>
#include <Encoder.h>

// הגדרת משתנים ופרמטרים
PIDController pid;
#define MOTOR_CCW 11 
#define MOTOR_CW 10 
#define enA 9 
#define ENCODER_A 2  
#define ENCODER_B 3 

float encoder_count; 
float enc_deg; 
int motor_pwm_value; 
int goal_degree = 0; // מצב התחלתי אפס מעלות
Encoder myEnc(ENCODER_B, ENCODER_A); 
float Kp = 12;  // פרמטר פרופורציונלי
float Ki = 0.85;  // פרמטר אינטגרלי
float Kd = 8;  // פרמטר נגזר

// המרה של טיקים למעלות
float tick_to_deg(long tick) {
  return (tick % 440) * 360.0 / 440.0;  // המרה של טיקים למעלות
}

// הפעלת המנוע בכיוון השעון
void forward() {
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
}

// הפעלת המנוע נגד כיוון השעון
void reverse() {
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, HIGH);
}

// קבלת קלט מהמשתמש
void get_user_input() {
  goal_degree = Serial.parseFloat();
  Serial.read();
  // שמירה על טווח בין -180 ל-180
  while (goal_degree > 180) goal_degree -= 360;
  while (goal_degree < -180) goal_degree += 360;
}

// אתחול ראשוני
void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);
  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);

  pid.begin();            // אתחול PID
  pid.limit(-255, 255);   // הגבלת פלט PID בין -255 ל-255
  pid.tune(Kp, Ki, Kd);   // הגדרת פרמטרי PID

  Serial.println("Please write the target angle between -180 and 180: ");
}

// משתנים למעקב אחרי מצב הזווית
bool isMoved = false; // משתנה למעקב אם המערכת הוזזה ידנית

void loop() {
  // אם יש קלט מהמשתמש (זווית יעד)
  if (Serial.available() > 0) {
    get_user_input();
    Serial.print("Goal angle set to: ");
    Serial.println(goal_degree);
  }

  encoder_count = myEnc.read();
  enc_deg = tick_to_deg(encoder_count);  // המרת טיקים למעלות

  // תיקון הזווית הנוכחית כך שתהיה בין -180 ל-180 מעלות
  if (enc_deg > 180) enc_deg -= 360;
  if (enc_deg < -180) enc_deg += 360;

  // הדפסת הזווית הנוכחית
  Serial.print("Current angle: ");
  Serial.println(enc_deg);

  // אם הזווית המצוינת על ידי הקוד היא 0, המנוע לא ינוע
  if (abs(enc_deg) < 45 && !isMoved) {
    // אם המערכת לא הוזזה ידנית עד 45 מעלות, אל תתחיל בתנועה
    Serial.println("Move manually until the system reaches 45 degrees.");
    return;
  }

  if (!isMoved && abs(enc_deg) >= 45) {
    // ברגע שהזווית תהיה מעל 45 מעלות, נסמן את המערכת ככזה שהוזזה ידנית
    isMoved = true;
    Serial.println("System has been moved. Now the motor will start moving to the target.");
  }

  // חישוב השגיאה בזווית
  float angle_error = goal_degree - enc_deg;

  // תיקון השגיאה כך שתהיה תמיד בין -180 ל-180
  if (angle_error > 180) angle_error -= 360;
  if (angle_error < -180) angle_error += 360;

  // הדפסת השגיאה
  Serial.print("Angle Error: ");
  Serial.println(angle_error);

  // אם השגיאה פחותה מ-15 מעלות, המנוע עוצר
  if (abs(angle_error) < 15) {
    analogWrite(enA, 0); // כיבוי המנוע
    Serial.println("Target reached");
    return;
  }

  // חישוב פלט PID
  motor_pwm_value = pid.compute(angle_error);

  // הגדרת כיוון התנועה
  if (motor_pwm_value > 0) {
    forward();
  } else {
    reverse();
  }

  analogWrite(enA, abs(motor_pwm_value));  // הגדרת מהירות המנוע

  delay(10);  // עיכוב למניעת עומס על המערכת
}
