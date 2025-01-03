#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// הגדרות רכיבים
#define BUTTON_PIN 2       // פין הכפתור
#define TRIG_PIN 3         // פין ה-TRIG של חיישן המרחק
#define ECHO_PIN 4         // פין ה-ECHO של חיישן המרחק
#define RST_PIN 10         // פין RST של RFID
#define SS_PIN 7           // פין SDA של RFID
#define SERVO_PIN 9        // פין של מנוע ה-Servo

// משתנים
LiquidCrystal_I2C lcd(0x27, 16, 2); // כתובת ה-LCD
MFRC522 rfid(SS_PIN, RST_PIN);      // יצירת אובייקט RFID
Servo servoMotor;                   // יצירת אובייקט מנוע Servo
bool rfidApproved = false;          // מעקב אחרי אישור RFID
bool itemTaken = false;             // מעקב אם פריט נלקח
int previousButtonState = LOW;      // מצב הכפתור הקודם

void setup() {
  // אתחול ה-LCD
  lcd.init();
  lcd.backlight();
  lcd.print("Welcome!");

  // אתחול RFID
  SPI.begin();
  rfid.PCD_Init();

  // הגדרת פינים
  pinMode(BUTTON_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);

  Serial.begin(9600);

  // הודעת התחלה
  delay(5000);
  lcd.clear();
  lcd.print("Scan RFID to");
  lcd.setCursor(0, 1);
  lcd.print("open the door");
}

void loop() {
  // בדיקת RFID
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String cardID = getCardID();
    if (cardID == "8A2C1AB1") { // מזהה כרטיס מותאם
      lcd.clear();
      lcd.print("Access Granted");
      rfidApproved = true;

      // פתיחת דלת
      servoMotor.write(90);
      delay(2000);
      servoMotor.write(0);
      delay(1000);

      lcd.clear();
      lcd.print("Approach to rack");
    } else {
      lcd.clear();
      lcd.print("Access Denied");
      delay(2000);
      lcd.clear();
      lcd.print("Scan RFID to");
      lcd.setCursor(0, 1);
      lcd.print("open the door");
    }
    rfid.PICC_HaltA();
  }

  // מדידת מרחק ולחיצת כפתור
  if (rfidApproved && !itemTaken) {
    int distance = getDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 20) { // פחות מ-2 מטר
      lcd.setCursor(0, 1);
      lcd.print("Press Button");

      // קריאת מצב הכפתור
      int buttonState = digitalRead(BUTTON_PIN);
      if (buttonState == HIGH && previousButtonState == LOW) {
        lcd.clear();
        lcd.print("Wait for rack");
        delay(2000);
        lcd.clear();
        lcd.print("Task complete!");
        lcd.setCursor(0, 1);
        lcd.print("Thank you!");
        delay(2000);

        // איפוס למצב התחלה
        rfidApproved = false;
        itemTaken = false;
        lcd.clear();
        lcd.print("Scan RFID to");
        lcd.setCursor(0, 1);
        lcd.print("open the door");
      }
      previousButtonState = buttonState;
    } else {
      lcd.setCursor(0, 1);
      lcd.print("                "); // ניקוי שורה שנייה ב-LCD
    }
  }
}

// פונקציה למדידת מרחק
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// פונקציה לקריאת מזהה ה-RFID
String getCardID() {
  String id = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    id += String(rfid.uid.uidByte[i], HEX);
  }
  id.toUpperCase();
  return id;
}
