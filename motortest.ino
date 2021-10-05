#include<Encoder.h>
#include<math.h>


int motorPin = 9;

Encoder myEnc(3, 5);


long int oldPosition  = -999;
double Kp = 0.21;
int given = 0;
double Ki = 0.172;
int intThreshholdCounts = 250;
unsigned long currentTime = 0;
double integral = 0;
int analog;
int closeEnough = 10;
long int newPosition;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
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




void loop() {
  long int newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  if ( digitalRead(11) == LOW && digitalRead(13) == LOW) {
    given = 0;
  }
  else if (digitalRead(11) == LOW && digitalRead(13) == HIGH) {
    given = 800;

  }
  else if (digitalRead(11) == HIGH && digitalRead(13) == LOW) {
    given = 1600;

  }
  else if (digitalRead(11) == HIGH && digitalRead(13) == HIGH) {
    given = 2400;
  }



  if (newPosition - given < 0 || newPosition - given > 0) {
    currentTime = millis();
    newPosition = myEnc.read();
    analog = Kp * (double)abs(newPosition - given) + integral * Ki;
    if (analog > 255) {
      analog = 255;
    }
    if (analog < 0) {
      analog = 0;
    }
    analogWrite(motorPin, analog);
    if (newPosition - given > 0) {
      digitalWrite(7, HIGH);
    }
    if (newPosition - given < 0) {
      digitalWrite(7, LOW);
    }
    if (abs(newPosition - given) < intThreshholdCounts) {
      integral += (double)abs(newPosition - given) * 0.05;
    }
    Serial.println((double)newPosition * (PI / 1600.0));
  }



  
  if (abs(newPosition - given) < intThreshholdCounts) {
    integral += (double)abs(newPosition - given) * 0.05;
  }
  else if (abs(newPosition - given) > intThreshholdCounts) {
    integral = 0;
  }
  if (abs(newPosition - given) < closeEnough) {
    integral = 0;
  }
  while (millis() < currentTime + 50);
}
