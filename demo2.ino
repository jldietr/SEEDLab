/* Arduino
   control speed and angular speed
*/

//TODO: Send in and interpret data from the Pi to manipulate the state variables and state helpers.

// libraries
#include <Encoder.h>
#include <math.h>
#include <Wire.h> //connects Pi to Arduino for LCD screen
#include <stdlib.h>
#include <string.h>

// system constants
#define SampleTime      40    // sampling time in milliseconds
#define MaxVoltage      7.5   // maximum voltage of the input into the motor
#define WheelRadius     0.0766   // radius of wheel in meters
#define WheelDistance   0.3048   // distance between wheels in meters
#define State2WaitTime 4000 //in milliseconds
#define Speed 70 //analogWrite value for constant velocity
#define measuringTime 10000 //in milliseconds
#define intThreshhold 6 // in meters
#define closeEnough 1.2 // in degrees

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

//Control variables
#define Ke                100
#define Kp                3.6
#define Ki                1.7

#define SLAVE_ADDRESS 0x04
char angleCHAR[6];
int angleRAW[6];
int zeroTrack;
//string angleName;
int angleRead;
int i = 0;
float angle;

// sets encoder functions
Encoder rightEnc(PinChannelRA, PinChannelRB);
Encoder leftEnc(PinChannelLA, PinChannelLB);

void receiveData(int byteCount);
bool fixAngle(float angle); //Going to be used in State 2 (Potentially 3 and 4)
void moveForward(float desVelocity);//Going to be used in State 3 and 4
void rotateInPlace(bool, double); //Going to be used in State 1

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

  //communications flags
  static bool tapeNotFound = 1;
  static bool correctAngle = 0;
  static bool commandToStop = 0;
  static bool commandToStop2 = 0;

  //communications data
  static double desiredTheta = 0;

  //system variables
  static double currentTime = 0;

  //system encoder-dependent data
  //Angles
  static double newDegreeLeft = 0;
  static double newDegreeRight = 0;
  static double oldDegreeLeft = 0;
  static double oldDegreeRight = 0;
  //Distance
  static double distanceRightWheelTravelled = 0;
  static double distanceLeftWheelTravelled = 0;
  static double errorPosition = 0;
  //Velocity
  static double velocityRight = 0;
  static double velocityLeft = 0;
  static double errorVelocity = 0;
  //Angular Velocity
  static double currentThetaRight = 0;
  static double currentThetaLeft = 0;
  static double errorTheta = 0;

  //system state flags
  static bool stateFindTape = 1;
  static bool stateFixAngle = 0;
  static bool stateMoveToStart = 0;
  static bool stateFixAngle2 = 0;
  static bool stateMoveToEnd = 0;

  //system state flag helpers
  static bool waitedLongEnough = 0;
  static bool waitedLongEnough2 = 0;
  

  //Time Reset
  currentTime = millis();

  //Reassign Angles
  newDegreeLeft = (double)(leftEnc.read() * ((360.0) / 3200.0));
  newDegreeRight = (double)(rightEnc.read() * ((360.0) / 3200.0));

  //Calculations of Position and Velocity
  distanceRightWheelTravelled =
    ((double) rightEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)
  distanceLeftWheelTravelled =
    -((double) leftEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)

  errorPosition = distanceRightWheelTravelled - distanceLeftWheelTravelled;

  currentThetaRight = RadiansToDegrees *
                      (distanceRightWheelTravelled / (WheelDistance * 0.5));
  currentThetaLeft = -RadiansToDegrees * (distanceLeftWheelTravelled / (WheelDistance * 0.5));
  errorTheta = currentThetaRight - currentThetaLeft;
  // Theta_sys(t) = X_wheel(t) / Radius_sys

  velocityRight = ((1000 * newDegreeRight - 1000 * oldDegreeRight)) / SampleTime;
  velocityLeft = -((1000 * newDegreeLeft - 1000 * oldDegreeLeft)) / SampleTime;

  errorVelocity = velocityRight - velocityLeft;

  //rotationalVelocity = errorVelocity / WheelDistance;








  //state Finding Tape
  //Tape is not found yet, feed 1 into the function
  if (stateFindTape && tapeNotFound) {
    rotateInPlace(tapeNotFound, errorVelocity);
  }
  //Once tape is found, tapeNotFound should be 0
  if (stateFindTape && !tapeNotFound) {
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
    stateFindTape = 0;
    stateFixAngle = 1;
    leftEnc.write(0);
    rightEnc.write(0);
  }





  //state Fix Angle to Tape
  if (stateFixAngle && !waitedLongEnough) {
    delay(State2WaitTime);
    waitedLongEnough = true;
  }
  if (stateFixAngle && waitedLongEnough) {
    if (correctAngle != true) {
      correctAngle = fixAngle(desiredTheta, currentThetaRight, currentThetaLeft, errorTheta);
    }
    if (correctAngle == true) {
      stateFixAngle = 0;
      stateMoveToStart = 1;
      leftEnc.write(0);
      rightEnc.write(0);
    }
  }







  //state moving to tape
  if (stateMoveToStart == true && !commandToStop) {
    digitalWrite(PinDirectionR, HIGH);
    digitalWrite(PinDirectionL, LOW);
    analogWrite(PinSpeedR, Speed - 5);
    analogWrite(PinSpeedL, Speed);
  }
  else if (stateMoveToStart && commandToStop) {
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
    stateMoveToStart = 0;
    stateFixAngle2 = 1;
    waitedLongEnough = 0;
  }








  //state fix angle at start of tape
  if (stateFixAngle2 && !waitedLongEnough) {
    delay(State2WaitTime);
    waitedLongEnough = true;
  }
  if (stateFixAngle2 && waitedLongEnough) {
    if (correctAngle != true) {
      correctAngle = fixAngle(desiredTheta, currentThetaRight, currentThetaLeft, errorTheta);
    }
    if (correctAngle == true) {
      stateFixAngle2 = 0;
      stateMoveToEnd = 1;
      leftEnc.write(0);
      rightEnc.write(0);
    }
  }







  //state moving to end of tape
  if (stateMoveToEnd && !waitedLongEnough2) {
    delay(measuringTime);
    waitedLongEnough2 = true;
  }
  if (stateMoveToEnd && !commandToStop2) {
    digitalWrite(PinDirectionR, HIGH);
    digitalWrite(PinDirectionL, LOW);
    analogWrite(PinSpeedR, Speed - 5);
    analogWrite(PinSpeedL, Speed);
  }
  else if (stateMoveToEnd && commandToStop2) {
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
    stateMoveToEnd = 0;
    leftEnc.write(0);
    rightEnc.write(0);
  }







  //Assign old angles before new loop
  oldDegreeLeft = newDegreeLeft;
  oldDegreeRight = newDegreeRight;

  // ensures function isn't taking too long
  if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");

  // creates delay of SampleTime ms
  while (millis() < (currentTime + SampleTime));

}





//END MAIN LOOP
//NOW ENTERING FUNCTIONS BELOW







void rotateInPlace(bool tapeNotFound, double errorVelocity) {

  int analogRight;
  int analogLeft;
  
  //If the tape has yet to be seen...
  if (tapeNotFound == true) {
    analogRight = Speed - Ke * errorVelocity;
    analogLeft = Speed + Ke * errorVelocity;

    digitalWrite(PinDirectionR, HIGH);
    digitalWrite(PinDirectionL, HIGH);
    analogWrite(PinSpeedR, analogRight);
    analogWrite(PinSpeedL, analogLeft);


  }
  //Once the tape is found...
  if (tapeNotFound == false) {
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
  }
}








bool fixAngle(double angle, double currentThetaRight, double currentThetaLeft, double errorTheta) {

  double desiredTheta = angle;
  double deltaThetaRight = desiredTheta - currentThetaRight;
  double deltaThetaLeft = desiredTheta - currentThetaLeft;
  bool correctAngle = 0;

  static double integralRight = 0;
  static double integralLeft = 0;
  
  int analogRight = Kp * (double)deltaThetaRight + integralRight * Ki - Ke * errorTheta;
  int analogLeft = Kp * (double)deltaThetaLeft + integralLeft * Ki + Ke * errorTheta;

  //Fix Upper and Lower Bounds for Analog
  if (analogRight > 100) {
    analogRight = 100;
  }
  if (analogRight < -100) {
    analogRight = -100;
  }

  if (analogLeft > 100) {
    analogLeft = 100;
  }
  if (analogLeft < -100) {
    analogLeft = -100;
  }

  //Determine Direction based on the wheel's analog value
  if (analogRight >= 0) {
    digitalWrite(PinDirectionR, HIGH);
  }
  if (analogRight < 0) {
    digitalWrite(PinDirectionR, LOW);
  }
  if (analogLeft >= 0) {
    digitalWrite(PinDirectionL, LOW);
  }
  if (analogLeft < 0) {
    digitalWrite(PinDirectionL, HIGH);
  }

  //Set voltage of each wheel
  analogWrite(PinSpeedL, abs(analogLeft));
  analogWrite(PinSpeedR, abs(analogRight));

  //Calculate integral right
  if (abs(deltaThetaRight) < intThreshhold) {
    integralRight += (double) abs(deltaThetaRight) * 0.04;
  }
  else if (abs(deltaThetaRight) > intThreshhold) {
    integralRight = 0;
  }
  if (abs(deltaThetaRight) < closeEnough) {
    integralRight = 0;
    correctAngle = true;
  }

  //Calculate integral left
  if (abs(deltaThetaLeft) < intThreshhold) {
    integralLeft += (double) abs(deltaThetaLeft) * 0.04;
  }
  else if (abs(deltaThetaLeft) > intThreshhold) {
    integralLeft = 0;
  }
  if (abs(deltaThetaLeft) < closeEnough) {
    integralLeft = 0;
    correctAngle = true;
  }
  return correctAngle;
}






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
