#include<Encoder.h>
#include<math.h>
#include<Wire.h>

#define SLAVE_ADDRESS 0x04
int data[32];


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

//MOTOR PIN SETUP:
//Red to MOTORSHIELD Vcc
//Black to MOTORSHIELD GND
//Green to ARDUINO GND
//Blue to ARDUINO Vcc
//Yellow to Arduino pin 3
//White to Arduino pin 5
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("Encoder Test:");

  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(sendData);
  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); //direction of motor
  pinMode(8, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  //raspberry pi input
  pinMode(11, INPUT);
  pinMode(13, INPUT);
  //raspberry pi output - for displaying desired position

}

void loop() {
  long int newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  
  //3200cnts/revolution on encoder
  if ( digitalRead(11) == LOW && digitalRead(13) == LOW) {
    given = 800; //pi/2 radians
    
  }
  else if (digitalRead(11) == LOW && digitalRead(13) == HIGH) {
    given = 1600; //pi radians
    
  }
  else if (digitalRead(11) == HIGH && digitalRead(13) == LOW) {
    given = 2400; //3pi/2 radians
    
  }
  else if (digitalRead(11) == HIGH && digitalRead(13) == HIGH) {
    given = 0; //0 radians
    
  }

  if (newPosition - given < 0 || newPosition - given > 0) {
    currentTime = millis();
    newPosition = myEnc.read();
    data[0] = newPosition / 20;
    analog = Kp * (double)abs(newPosition - given) + integral * Ki; //movement speed of motor
    if (analog > 255) {
      analog = 255;
    } //limiter for out of bounds for output of controller
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

  //integral calculation
  if (abs(newPosition - given) < intThreshholdCounts) {
    integral += (double)abs(newPosition - given) * 0.05;
  }
  else if (abs(newPosition - given) > intThreshholdCounts) {
    integral = 0; //zero out the integral when we're close enough to desired position
  }
  if (abs(newPosition - given) < closeEnough) {
    integral = 0;
  }
  while (millis() < currentTime + 50); //timer for consistency of next control input calculation - 50ms
}

void sendData(){
  Wire.write(data[0]);
}
