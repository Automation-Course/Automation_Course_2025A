#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define RedLed 13
#define GreenLed 11
#define ServoPin 9
#define YellowLed 12
#define PIRsensor A2

Servo servo;
LiquidCrystal_I2C lcd_1(0X27, 16, 2);

int angle = 0;
int speed = 90;
int buttonState = 0;
int button = 4;
int pir_sensor = 0;
int pir_sensor_map = 0;
bool pirState = false;
int speakerPin = 7;



// Happy Birthday song
int length = 28; // the number of notes
char notes[] = "GGAGcB GGAGdc GGxecBA yyecdc";
int beats[] = { 2, 2, 8, 8, 8, 16, 1, 2, 2, 8, 8,8, 16, 1, 2,2,8,8,8,8,16, 1,2,2,8,8,8,16 };
int tempo = 150;

void playTone(int tone, int duration) {


for (long i = 0; i < duration * 1000L; i += tone * 2) {

   digitalWrite(speakerPin, HIGH);

   delayMicroseconds(tone);

   digitalWrite(speakerPin, LOW);

   delayMicroseconds(tone);

}

}

void playNote(char note, int duration) {

char names[] = {'C', 'D', 'E', 'F', 'G', 'A', 'B',           



                 'c', 'd', 'e', 'f', 'g', 'a', 'b',



                 'x', 'y' };



int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014,



                 956,  834,  765,  593,  468,  346,  224,



                 655 , 715 };



int SPEE = 5;



// play the tone corresponding to the note name


for (int i = 0; i < 17; i++) {

   if (names[i] == note) {

    int newduration = duration/SPEE;

     playTone(tones[i], newduration);

   }

}



}


void setup()  
{
  lcd_1.init();  
  lcd_1.backlight();
  pinMode(RedLed, OUTPUT);
  pinMode(YellowLed, OUTPUT);
  pinMode(GreenLed, OUTPUT);
  pinMode(button, INPUT);
  pinMode(PIRsensor, INPUT);
  pinMode(speakerPin, OUTPUT);
  Serial.begin(9600);
  servo.attach(ServoPin);
} 

void loop() {
 
    pirState = movement_pir(); // check if there is motion

  if (pirState == true ) { // if motion is detected
   
    lcd_1.print("BIENVENIDO!");
    digitalWrite(RedLed, HIGH);
    digitalWrite(YellowLed, HIGH);
    digitalWrite(GreenLed, HIGH);
    servo.attach(ServoPin);
    speed = 180;
    servo.write(speed);
    delay(3000);
    digitalWrite(RedLed, LOW);
    digitalWrite(YellowLed, LOW);
    digitalWrite(GreenLed, LOW);
  } else {
    lcd_1.clear();
    Serial.println("No motion");
     digitalWrite(RedLed, LOW);
    digitalWrite(YellowLed, LOW);
    digitalWrite(GreenLed, LOW);
    servo.write(90);
  }

  
    buttonState = digitalRead(button); // check if button pressed
  if (buttonState == HIGH) {
    
      lcd_1.clear(); // clear screen
      lcd_1.setCursor(0, 0); // define first row
      lcd_1.print("feliz cumpleanos");
      Serial.println("feliz cumpleanos");
      
      // Turn on lights
      digitalWrite(RedLed, HIGH);
      digitalWrite(YellowLed, HIGH);
      digitalWrite(GreenLed, HIGH);
    
        // play Happy Birthday song

      for (int i = 0; i < length; i++) {
  
   if (notes[i] == ' ') {

     delay(beats[i] * tempo); // rest

   } else {

     playNote(notes[i], beats[i] * tempo);

   }


   // pause between notes

   delay(tempo);

}
      delay(3000);
      digitalWrite(RedLed, LOW);
      digitalWrite(YellowLed, LOW);
      digitalWrite(GreenLed, LOW);
  } 
 
}
  

// check if there is motion
bool movement_pir() {
  pir_sensor = analogRead(PIRsensor);
  Serial.print("Mapped PIR Sensor Value: ");
  Serial.println(pir_sensor_map);
  pir_sensor_map = map(pir_sensor, 0, 1023, 0, 255);
  if (pir_sensor_map > 85) {
    Serial.println("Motion Detected!");
    delay(100);
    return true;
  }
      lcd_1.clear();
  return false;
}
