/* Arduino
   control speed and angular speed
*/

// libraries
#include <Encoder.h>
#include <math.h>
#include <Wire.h> //connects Pi to Arduino for LCD screen
#include <stdlib.h>
#include <string.h>

#define SLAVE_ADDRESS 0x04
char angleCHAR[6];
int angleRAW[6];
int zeroTrack;
//string angleName;
int angleRead;
int i = 0;
float angle;

// system constants
#define SampleTime      40    // sampling time in milliseconds
#define MaxVoltage      7.5   // maximum voltage of the input into the motor
#define WheelRadius     0.0766   // radius of wheel in meters
//TODO: Measure wheelbase for now assume 1/3 of a meter
#define WheelDistance   0.3048   // distance between wheels in meters

// conversion constants
#define DegreesToRadians   0.01745329
#define MeterInFeet        0.3048

// pins
#define PinChannelRA      2     // encoder input pin for right motor channel A
#define PinChannelLA      3     // encoder input pin for left motor channel A
#define PinChannelRB      13    // encoder input pin for right motor channel B
#define PinChannelLB      6     // encoder input pin for left motor channel B
#define Enable              4 //enable motor
#define PinDirectionR     7     // right motor direction
#define PinDirectionL     8     // left motor direction
#define PinSpeedR         9    // PWM pin for right motor speed
#define PinSpeedL         10    // PWM pin for left motor speed    
#define RadiansToDegrees 180/PI

#define intThreshhold 6
#define closeEnough 1.2


// sets encoder functions
Encoder rightEnc(PinChannelRA, PinChannelRB);
Encoder leftEnc(PinChannelLA, PinChannelLB);

// global variables
/*
  actual vs desired + error
  position distnace, position rotation
  speed distance, speed rotation
*/

double desiredTheta = 0;

double errorPosition = 0;



double Ke = 0.05;
double Kp = 3.8;
double Ki = 1.1;


void receiveData(int byteCount);

void setup() {

  // serial communication initialization
  Serial.begin(115200);

  // assigns pins as either inputs or outputs
  pinMode(Enable, OUTPUT);
  pinMode(PinDirectionR, OUTPUT);
  pinMode(PinDirectionL, OUTPUT);
  pinMode(PinSpeedR, OUTPUT);
  pinMode(PinSpeedL, OUTPUT);

  pinMode(12, INPUT_PULLUP);

  // enables motors
  digitalWrite(Enable, HIGH);

  // writes direction to motor
  digitalWrite(PinDirectionR, LOW);
  digitalWrite(PinDirectionL, LOW);

  // Pi connection
    Wire.begin(SLAVE_ADDRESS); //sends data to LCD
   Wire.onReceive(receiveData);
    
} // end of setup


double distanceRightWheelTravelled = 0;
double distanceLeftWheelTravelled = 0;
double currentTheta = 0;
double integral = 0;
int analog = 0;
int deltaTheta = 0;
unsigned long currentTime = 0;
bool correctAngle = 0;
double deltaPosition = 0;
double desiredPosition = 0;
double integralMove = 0;
double Kmovep = 300;
double Kmovei = 70;
double currentPosition = 0;
double moveCloseEnough = 0.005;
double intMoveThreshhold = 0.5;
bool switchAdjustPos = true;
double adjuster = 0;
bool gotime = 0;

void loop() {

  // static variables

   static double newDegreeLeft = 0;
  static double newDegreeRight = 0;
  static double oldDegreeRight = 0;
   static double oldDegreeLeft = 0;

  




  // measures time for delay
  currentTime = millis();

  // take sample of position,calculate position, calculate speed

  //Serial.println(currentPosition);

  distanceRightWheelTravelled =
    ((double) rightEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)
  distanceLeftWheelTravelled =
    ((double) leftEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)

  errorPosition = distanceRightWheelTravelled - distanceLeftWheelTravelled;

  currentTheta = -RadiansToDegrees *
                 (distanceRightWheelTravelled / (WheelDistance * 0.5)) / 2 + -RadiansToDegrees * (distanceLeftWheelTravelled / (WheelDistance * 0.5)) / 2; // Theta_sys(t) = X_wheel(t) / Radius_sys


  currentPosition = (distanceRightWheelTravelled/2 - distanceLeftWheelTravelled/2);


  // calculate angular velocity of wheels
 // velocityRight = (1000 * (newDegreeRight - oldDegreeRight)) / SampleTime;
  //velocityLeft = (1000 * (newDegreeLeft - oldDegreeLeft)) / SampleTime;

  // calculate speed of system
  //actualXY = WheelRadius * 0.5 * (newDegreeRight + newDegreeLeft) * DegreesToRadians;
  //actualRotation = (WheelRadius / WheelDistance) * (newDegreeLeft - newDegreeRight);

  // get desired angle by converting array of chars to float
 
  angle = atof(angleCHAR);
 
  //angle = 18.43;
  desiredTheta = angle;
  deltaTheta = currentTheta - desiredTheta;

  
  desiredPosition = 1;
  deltaPosition = desiredPosition - currentPosition;
   Serial.println(desiredTheta);
  //determine voltage

if(digitalRead(12) == HIGH) {
  gotime = true;
}

  if(gotime == true){
  
  if (deltaTheta != 0 && correctAngle == false) {

   
    analog = Kp * (double) abs(deltaTheta) + integral * Ki;
    if (analog > 255) {
      analog = 255;
    }
    if (analog < 0) {
      analog = 0;
    }
    if ( errorPosition >= 0 ) {
      analogWrite(PinSpeedR, analog);
      analogWrite(PinSpeedL, analog);
    }
    if ( errorPosition < 0 ) {
      analogWrite(PinSpeedL, analog);
      analogWrite(PinSpeedR, analog);
    }
    if (deltaTheta < 0) {
      digitalWrite(PinDirectionR, HIGH);
      digitalWrite(PinDirectionL, HIGH);
    }
    if (deltaTheta > 0) {
      digitalWrite(PinDirectionR, LOW);
      digitalWrite(PinDirectionL, LOW);
    }


    if (abs(deltaTheta) < intThreshhold) {
      integral += (double) abs(deltaTheta) * 0.04;
    } else if (abs(deltaTheta) > intThreshhold) {
      integral = 0;
    }
    if (abs(deltaTheta) < closeEnough) {
      integral = 0;
    }

    
    if (abs(deltaTheta) < closeEnough) {
      correctAngle = true;
    }
  }

  if(switchAdjustPos && correctAngle){
    adjuster = currentPosition;
    switchAdjustPos = false;
    Serial.println("Adjusting");
    
  }
  currentPosition -= adjuster;
   if(deltaPosition != 0 && correctAngle == true){
    //Serial.println("moving forward");
    analog = Kmovep * (double)abs(deltaPosition) + Kmovei * integralMove;
    if ( analog > 255) {
      analog = 255;
    }
    if ( analog < 0 ) {
      analog = 0;
    }
    analogWrite(PinSpeedR, analog);
    analogWrite(PinSpeedL, analog);
    digitalWrite(PinDirectionR, HIGH);
    digitalWrite(PinDirectionL, LOW);

    if ( abs(deltaPosition) < intMoveThreshhold) {
      integralMove += (double)abs(deltaPosition) * 0.04;
    }
    else if (abs(deltaPosition) > intMoveThreshhold) {
      integralMove = 0;
    }
    if ( abs(deltaPosition) < moveCloseEnough) {
      integralMove = 0;
    }
    
   }

  }
  // Serial.println(rightEnc.read());
  //Serial.println(leftEnc.read());
  // reassign angles

  // ensures function isn't taking too long
  if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");

  // creates delay of SampleTime ms
  while (millis() < (currentTime + SampleTime));

} // end of loop





void receiveData(int byteCount) {

  while (Wire.available()) {
    angleRead = Wire.read();
    if (angleRead != 0) {
      Serial.print("data received: ");
      Serial.println(angleRead);
      angleRAW[i] = angleRead;
      angleCHAR[i] = angleRAW[i];
      //angleName += angleCHAR[i];
      i++; // need to reset i once we have the angle
      zeroTrack = 0;
      if (i == 6) {
        i = 0;
      }
    }
    if (angleRead == 0) {
      zeroTrack++;
      if (zeroTrack == 2) {
        break;
      }
    }
  }
}
