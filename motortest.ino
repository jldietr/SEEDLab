#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_2 true


#include<Encoder.h>
#include<math.h>
#include<TimerInterrupt.h>

int motorPin = 9;
unsigned long time_now = 0;
int period = 50;
int PWMvalue = 200;


Encoder myEnc(3, 5);


long int oldPosition  = -999;
double Kp = 0.21;
int given = 0;
double Ki = 0.172;
double Kd = 0;
bool lastFlag;
int intThreshholdCounts = 250;
unsigned long currentTime = 0;
double integral = 0;
int analog;
int closeEnough = 10;
long int newPosition;

void controller() {
  newPosition = myEnc.read();
  if (newPosition - given > 0) {
    digitalWrite(7, HIGH);
  }
  if (newPosition - given < 0) {
    digitalWrite(7, LOW);
  }
  analog = Kp * (double)abs(newPosition - given) + integral * Ki;
  if (analog > 255) {
    analog = 255;
  }
  analogWrite(motorPin, analog);
  if (abs(newPosition - given) < intThreshholdCounts) {
    integral += (double)abs(newPosition - given) * 0.05;
  }
  else if (abs(newPosition - given) > intThreshholdCounts) {
    integral = 0;
  }
  if (abs(newPosition - given) < closeEnough) {
    integral = 0;
  }
}

#define TIMER_INTERVAL_MS 50


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

  ITimer2.init();
  // interval (in ms) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely


   // Interval in unsigned long millisecs
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS, controller))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

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
 /* while (newPosition - given < 0 || newPosition - given > 0) {

    currentTime = millis();
    newPosition = myEnc.read();
    analog = Kp * (double)abs(newPosition - given) + integral * Ki;
    if (analog > 255) {
      analog = 255;
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
    while (millis() < currentTime + 50);
  }
  integral = 0;*/

}
