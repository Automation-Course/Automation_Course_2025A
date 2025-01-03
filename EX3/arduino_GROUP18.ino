#include <DHT.h>


#define DHTPIN A0       // Pin connected to the DHT11 data pin
#define DHTTYPE DHT11  // DHT 11 sensor

DHT dht(DHTPIN, DHTTYPE);
const int touchPin = 13;  // Touch sensor connected to pin 13
const int ledPin = 5;  
int power =0;   
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
#include <Servo.h>
#define servoPin  9
Servo servo;
int angle = 0;
bool Direction = true; 
int continues=0;
int continues2=0;
int buttonState = 0;       // מצב הכפתור הנוכחי
int lastButtonState = 0;   // מצב הכפתור הקודם
int ledState = LOW;
int ledState2 = LOW;   // מצב ה-LED (מתחיל מכובה)
int counter =0;
int incoming=0;
#define led  13            // פין של ה-LED
#define inPin 2            // פין של הכפתור
#define led2 11
#define piezoPin 3

int flag=0;
unsigned long pushedtime = 0;
unsigned long lastDebounceTime = 0;  // הזמן האחרון בו הכפתור שונה
unsigned long debounceDelay = 50;// זמן דיבאונס (עיכוב כדי למנוע רעשים)
unsigned long pushDelay=1500;
unsigned long waitingForWaitress=7000;
unsigned long lightTime=0;

void setup() {
  pinMode(inPin, INPUT);  // הגדרת הפין של הכפתור כקלט

  pinMode(led2, OUTPUT);// הגדרת הפין של ה-LED כיציאה
  Serial.begin(9600);
  servo.attach(servoPin);
  pinMode(touchPin, INPUT);  // Set touchPin as input
  pinMode(ledPin, OUTPUT);   // Set ledPin as output
  pinMode(piezoPin, INPUT);
  Serial.begin(9600); 
  lcd.init();
  analogWrite(piezoPin,LOW);
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Welcome");
  Serial.println("Welcome");
  dht.begin();
}

void loop() {
  int reading = digitalRead(inPin);  // קריאה למצב הכפתור
  if(reading ==HIGH){
  if (millis() - pushDelay > pushedtime )
    pushedtime = millis();
  if (millis() - pushDelay < pushedtime and reading != lastButtonState){
    counter++;
    
    
    lastButtonState = reading;
  }
  }
  if (reading == LOW)
    lastButtonState = LOW;
  if(millis() - pushDelay > pushedtime )
  {
    if (counter ==1 )
    {
      ledState = (ledState == LOW) ? HIGH : LOW;
      Serial.println("The button was pressed 1 time");
      Serial.println("The servo changed state");
      

      if (ledState == LOW)
        servo.write(90);  
    }
    if (counter ==2)
    {
      ledState2 = (ledState2 == LOW) ? HIGH : LOW;
      Serial.println("The button was pressed 2 times");
      digitalWrite(led2, ledState2);
      Serial.println("The light bulb1 changed state");
      
    }
    if(counter ==3)
    {
      power =0;
      Serial.println("The button was pressed 3 times");
      analogWrite(ledPin, power);
      Serial.println("The light bulb2 was turned off");
      
    }
    if (counter==4)
    {
      // Read temperature and humidity
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
  
    // Check if any reading failed
    if (isnan(humidity) || isnan(temperatureC)) {
      Serial.println("Failed to read from DHT sensor!");
   } else {
     Serial.println("The button was pressed 4 times");
      // Print the values to the Serial Monitor
      Serial.print("Temperature: ");
     Serial.print(temperatureC);
      Serial.println(" °C ");
      lcd.setCursor(0, 0);
      lcd.print("Temperature:    ");
      lcd.setCursor(0, 1);
      lcd.print(temperatureC);
      
    }
    }
    counter =0;
  }
 
  if (millis()-waitingForWaitress<lightTime)
  {
   if (incoming == 0)
   {
     lcd.setCursor(0, 0);
     lcd.print("waitress on the");
     Serial.println("waitress on the way");
     lcd.setCursor(0, 1);
     lcd.print("way");
     incoming =1;
     lastButtonState= LOW;
   }
  }
  if (millis()-waitingForWaitress>lightTime and incoming == 1)
  {
    digitalWrite(led2, LOW);
    Serial.println("The light bulb1 was turned off");
    ledState2 = LOW;
    incoming=0;
  }
 
  if (millis()-waitingForWaitress>lightTime and digitalRead(led2)==HIGH)
    lightTime=millis();
  if (ledState == HIGH)
  {
    if (Direction){ 
      angle +=1;
       }else{
       angle -=1; 
      }
      servo.write(angle);
       delay(15);
      if(angle == 180)Direction = false; 
      if(angle == 0)Direction = true;
  }
   int touchState = digitalRead(touchPin);  // Read touch sensor state
   if (continues==0 and touchState == HIGH)
    {
    Serial.println("the touchButton is pushed ");
    continues=1;
    }
  if (touchState == HIGH and power <255) {
    power++;
    
    
   analogWrite(ledPin, power);  // Turn LED on if touch is detected
   delay(10);
  } 
  else
  if (touchState == HIGH)
  {
    power=255;
    analogWrite(piezoPin,HIGH);
    if(continues2==0)
    {
      Serial.println("the piezo is on");
      continues2=1;   
    }
    flag=1;
    
  }
  if (digitalRead(touchPin)==LOW)
    continues=0;
  if (digitalRead(touchPin)==LOW and flag==1)
  {
    Serial.println("the piezo is off ");
    analogWrite(piezoPin,LOW);
    flag=0;
    continues2=0;
  }
}


