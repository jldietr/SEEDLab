#include<Encoder.h>
#include<math.h>

int motorPin = 9;
unsigned long time_now = 0;
int period = 50;
int PWMvalue = 200;


Encoder myEnc(3,5);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Encoder Test:");
  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  
  pinMode(11, INPUT);
  pinMode(13, INPUT);
  
  
}



long oldPosition  = -999;
double Kp = 0.11;
int given;
double Ki = 0;
double Kd = 0;
bool lastFlag;
int intThreshholdCounts = 100 ;
int currentTime = 0;
double integral = 0;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

    if ( digitalRead(11) == LOW && digitalRead(13) == LOW) {
      given = 0;
    }
    if (digitalRead(11) == LOW && digitalRead(13) == HIGH){
      given = 800;

    }
    if(digitalRead(11) == HIGH && digitalRead(13) == LOW){
      given = 1600;

    }
    if(digitalRead(11) == HIGH && digitalRead(13) == HIGH){
      given == 2400;

    }
      while(newPosition-given != 0) {
        currentTime == millis();
        Serial.println(Kp*abs(newPosition-given));
         analogWrite(motorPin, (Kp)*abs(newPosition-given)+integral*Ki);
         if ( newPosition - given > 0) {
          digitalWrite(7, HIGH);
         }
         if(newPosition - given < 0) {
          digitalWrite(7, LOW);
         }
         if(abs((newPosition-given)) < intThreshholdCounts){
         integral += abs(newPosition-given)*0.05;
         }
         newPosition = myEnc.read();
         Serial.println(newPosition);
         while(millis() < currentTime + 50);
    }
    Kp = 0;
    
}
