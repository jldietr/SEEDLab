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

double desiredTheta = 0;
double errorPosition = 0;
double Ke = 0;
double Kp = 3.3;
double Ki = 1.1;
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
bool stopRead = false;
bool gotime = false;
bool buttonState = 0;
bool switchTime = 0;
double velocityRight = 0;
double velocityLeft = 0;
double errorVelocity = 0;
double newDegreeLeft = 0;
double newDegreeRight = 0;
double oldDegreeRight = 0;
double oldDegreeLeft = 0;

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

  pinMode(11, INPUT);

  // enables motors
  digitalWrite(Enable, HIGH);

  // writes direction to motor
  digitalWrite(PinDirectionR, LOW);
  digitalWrite(PinDirectionL, LOW);

  // Pi connection
  Wire.begin(SLAVE_ADDRESS); //sends data to LCD
  Wire.onReceive(receiveData);

} // end of setup
   
void loop() {

  // static variables

  static double analogLeft = 0;
  static double analogRight = 0;

  currentTime = millis();

  newDegreeLeft = (double)(leftEnc.read() * ((360.0) / 3200.0));
  newDegreeRight = (double)(rightEnc.read() * ((360.0) / 3200.0));

  //Serial.println(currentPosition);

  distanceRightWheelTravelled =
    ((double) rightEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)
  distanceLeftWheelTravelled =
    ((double) leftEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)

  errorPosition = distanceRightWheelTravelled + distanceLeftWheelTravelled;
  //Serial.println(errorPosition);

  currentTheta = -RadiansToDegrees *
                 (distanceRightWheelTravelled / (WheelDistance * 0.5)) / 2 + -RadiansToDegrees * (distanceLeftWheelTravelled / (WheelDistance * 0.5)) / 2; // Theta_sys(t) = X_wheel(t) / Radius_sys


  currentPosition = (distanceRightWheelTravelled / 2 - distanceLeftWheelTravelled / 2);


  // calculate angular velocity of wheels
  velocityRight = (1000 * (newDegreeRight - oldDegreeRight)) / SampleTime;
  velocityLeft = (1000 * (newDegreeLeft - oldDegreeLeft)) / SampleTime;

  // calculate difference in velocity between right and left wheels
  errorVelocity = velocityRight + velocityLeft;
  

  // get desired angle by converting array of chars to float
  if ( stopRead == false) {
     // When the Pi does not see an angle it will send "180.00" as it will never be a real angle sent by the Pi
     // We need to account for this in the motor controller so that it steps until it gets a real angle
    //angle = atof(angleCHAR);
    angle = -116.6;
  }

  desiredTheta = angle;
  deltaTheta = currentTheta - desiredTheta;

  Serial.println(errorPosition);
  
  desiredPosition = 0.6096 + 0.045;
  deltaPosition = desiredPosition - currentPosition;
  // Serial.println(desiredTheta);
  //determine voltage

   //button to reset the robot so it can try to find a new angle
  buttonState = digitalRead(11);
  //Serial.println(switchTime);
  if (buttonState == true) {
    switchTime = !switchTime;
    if (switchTime == true) {
      gotime = true; //flag to start moving towards an angle
      stopRead = true; // flag to stop trying to read a new angle from the Pi
    }
    if (switchTime == false) {
      gotime = false;
      stopRead = false;
      correctAngle = false;
      switchAdjustPos = true;
      analogWrite(PinSpeedR, 0);
      analogWrite(PinSpeedL, 0);
    }
    rightEnc.write(0);
    leftEnc.write(0);
    delay(200);
  }

  if (gotime == true) {

    if (deltaTheta != 0 && correctAngle == false) {

      Serial.println("turning");
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
    if(deltaTheta == 0) {
      correctAngle = true;
    }

    if (switchAdjustPos && correctAngle) {
      leftEnc.write(0);
      rightEnc.write(0);
      switchAdjustPos = false;
      Serial.println("Adjusting");
      delay(400);
    }
     
     // angle corrections
    if (deltaPosition != 0 && correctAngle == true) {
      //Serial.println("moving forward");
      analog = Kmovep * (double)deltaPosition + Kmovei * integralMove;
      analogRight = analog + Ke*errorPosition;
      analogLeft = analog - Ke*errorPosition;
      if ( analogRight > 255) {
        analogRight = 170;
      }
      if ( analogRight < -255 ) {
        analogRight = -170;
      }
      if ( analogRight >= 0 ) {
        digitalWrite(PinDirectionR, HIGH);
      }
      if (analogRight < 0 ) {
        digitalWrite(PinDirectionR, LOW);
      }
         if ( analogLeft > 255) {
        analogLeft= 175;
      }
      if ( analogLeft < -255 ) {
        analogLeft = -175;
      }
      if ( analogLeft >= 0 ) {
        digitalWrite(PinDirectionL, LOW);
      }
      if (analogLeft < 0 ) {
        digitalWrite(PinDirectionL, HIGH);
      }
      
      
      analogWrite(PinSpeedR, abs(analogRight));
      analogWrite(PinSpeedL, abs(analogLeft));

      Serial.println(analogRight + Ke * errorPosition);
      Serial.println(analogLeft - Ke * errorPosition);
      //Serial.println(deltaPosition);
//      if ( analog >= 0 ) {
//        digitalWrite(PinDirectionR, HIGH);
//        digitalWrite(PinDirectionL, LOW);
//      }
//      if (analog < 0 ) {
//        digitalWrite(PinDirectionR, LOW);
//        digitalWrite(PinDirectionL, HIGH);
//      }
      

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
  oldDegreeLeft = newDegreeLeft;
  oldDegreeRight = newDegreeRight;
   
  // ensures function isn't taking too long
  if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");

  // creates delay of SampleTime ms
  while (millis() < (currentTime + SampleTime));

} // end of loop

/* we send ASCII characters from the Pi to the Arduino. If the ASCII chracter equals 0 it is ignored.
   the ASCII chracters is inserted into an array of chars, once we have the entire angle (e.g. -41.58 deg) the whole array is converted to a float.
   This float is the angle the robot rotates too.
*/

void receiveData(int byteCount) {

  while (Wire.available()) {
    angleRead = Wire.read();
    if (angleRead != 0) {
      //      Serial.print("data received: ");
      //      Serial.println(angleRead);
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
