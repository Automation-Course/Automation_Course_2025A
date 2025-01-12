#include <Encoder.h>
#include <PIDController.h>

// הגדרות רכיבים
#define MOTOR_CCW 11
#define MOTOR_CW 10
#define ENCODER_A 2
#define ENCODER_B 3
#define enA 9


// משתנים גלובליים
PIDController pid;
Encoder myEnc(ENCODER_B, ENCODER_A);

int encoder_count = 0;
float tick_to_deg1 = 0;
int motor_pwm_value = 0;
float Kp = 10 , Ki = 0.2, Kd = 7; 
float goal_deg = 0.0;
bool new_goal_set = false; // משתנה לעדכון קלט חדש

// פונקציות עזר
float tick_to_deg(long tick) {
    return tick * 360.0 / 440.0; // עדכון לפי האנקודר שלך
}

void forward() {
    digitalWrite(MOTOR_CW, HIGH);
    digitalWrite(MOTOR_CCW, LOW);
}

void reverse() {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, HIGH);
}

void enterDeg() {
    goal_deg = Serial.parseFloat();
    Serial.read(); // נקה את ה-buffer
    if (goal_deg < -15 || goal_deg > 15) {
        Serial.println("Error: Enter a degree between -15 and 15.");
    } else {
        pid.setpoint(goal_deg);
        new_goal_set = true; // עדכן שהוזן ערך חדש
        Serial.print("New goal set: ");
        Serial.println(goal_deg);
    }
}

void setup() {
    Serial.begin(9600);

    // הגדרות פינים
    pinMode(enA, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);

    // כיוון מנוע התחלתי
    forward();

    // הגדרות PID
    pid.begin();
    pid.limit(-255, 255);
    pid.tune(Kp, Ki, Kd);
    pid.setpoint(goal_deg);

    Serial.println("Enter the goal degree (-15 to 15):");
}

void loop()
 {
   // בדיקת קלט מהמשתמש
    if (Serial.available() > 0) 
    {
        enterDeg();
    }

    // קריאת אנקודר
    encoder_count = myEnc.read();
    tick_to_deg1 = tick_to_deg(encoder_count);

    // שמירה על הזווית בתחום -360 עד 360 מעלות
    while 
    (abs(tick_to_deg1) > 360) {
        if (tick_to_deg1 < 0) {
            tick_to_deg1 += 360;
        } else {
            tick_to_deg1 -= 360;
        }
    }

    // חישוב ערך PID
    motor_pwm_value = float(pid.compute(tick_to_deg1));

    // קביעת כיוון המנוע
    if (motor_pwm_value < 0) {
        reverse();
    } else {
        forward();
    }

    // שליחת PWM למנוע
    analogWrite(enA, abs(motor_pwm_value));

    // הצגת נתונים רק אם הוזן ערך חדש
    if (new_goal_set) {
        Serial.print("Current Angle: ");
        Serial.println(tick_to_deg1);
        Serial.print("Goal Angle: ");
        Serial.println(goal_deg);
        new_goal_set = false; // אפס את המצב לאחר ההדפסה
    }

     delay(10); // עיכוב קטן ליציבות{}
}