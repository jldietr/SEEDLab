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
#define WheelRadius     0.0756   // radius of wheel in meters
#define WheelDistance   0.3048   // distance between wheels in meters
#define State2WaitTime 2000 //in milliseconds
#define Speed 70 //analogWrite value for constant velocity
#define measuringTime 10000 //in milliseconds
#define intThreshhold 6 // in meters
#define closeEnough 1.2 // in degrees
#define intVelThreshhold 0.2 //in m/s
#define closeEnoughVel 0.01 //in m/s

// conversion constants
#define DegreesToRadians   PI/180
#define MeterInFeet        0.3048

// pins
#define PinChannelRA      2     // encoder input pin for right motor channel A
#define PinChannelLA      3     // encoder input pin for left motor channel A
#define PinChannelRB      13    // encoder input pin for right motor channel B
#define PinChannelLB      6     // encoder input pin for left motor channel B
#define Enable              4 //enable motor
#define PinDirectionR     8     // right motor direction
#define PinDirectionL     7     // left motor direction
#define PinSpeedL         9    // PWM pin for right motor speed
#define PinSpeedR         10    // PWM pin for left motor speed    
#define RadiansToDegrees 180/PI

//Control variables
#define Ke                0
#define Kp                0.01
#define Ki                0.0
#define Kd                1.0

#define KeVel             0
#define KpVel             15.0
#define KiVel             0

#define SLAVE_ADDRESS 0x04
char angleCHAR[6];
int angleRAW[6];
int zeroTrack;
//string angleName;
int angleRead;
int i = 0;
double angle;
/*TODO:
   convert angle to float
   if angle is 180deg keep turning -> tapeNotFound=1 && rotateInPlace
   once blue tape is found interpret angle [-30deg,30deg]-> tapeNotFound=0 && stateFixAngle =1
   if angle == [-30deg,30deg] rotate robot until angle ==0
   when angle ==0 angle == 999
   999 is signal to change states and move forward
   whne close to tape angle == 777
*/

// sets encoder functions
Encoder rightEnc(PinChannelRA, PinChannelRB);
Encoder leftEnc(PinChannelLA, PinChannelLB);

//Functions declarations
void receiveData(int byteCount);
bool fixAngle(double angle, double currentThetaRight, double currentThetaLeft, double errorTheta, double tanVelocityRight, double tanVelocityLeft); //Going to be used in State 2 (Potentially 3 and 4)
void moveForward(double desVelocity, double currentVelocityRight, double currentVelocityLeft, double desTurnRate);//Going to be used in State 3 and 4
void rotateInPlace(bool, double); //Going to be used in State 1

double turnRateConverter(double angle);

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
  //static bool commandToStop = 0;

  //controller command and reset flags
  static bool buttonState = 0;
  static bool goTime = 0;
  static bool switchTime = 0;


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
  //Distance Variables
  static double distanceRightWheelTravelled = 0;
  static double distanceLeftWheelTravelled = 0;
  static double errorPosition = 0;
  //Velocity Variables
  static double velocityRight = 0;
  static double velocityLeft = 0;
  static double tanVelocityRight = 0;
  static double tanVelocityLeft = 0;
  static double errorVelocity = 0;
  static double errorTanVelocity = 0;
  //Angular Position Variables
  static double currentThetaRight = 0;
  static double currentThetaLeft = 0;
  static double errorTheta = 0;

  //system state flags
  static bool stateFindTape = 0; //<---- SET THIS TO 1 TO START THE SEQUENCE
  static bool stateFixAngle = 0;
  static bool stateMoveToStart = 0;
  static bool stateFixAngle2 = 0;
  static bool stateMoveToEnd = 0;

  //system state flag helpers
  static bool waitedLongEnough = 0;
  static bool waitedLongEnough2 = 0;


  //Tester Values for Functions
  static double desVelocity = 0.106;
  static double desTurnRate = WheelDistance * DegreesToRadians * 40;
  currentTime = millis();
  //Read angle from comms
  angle = atof(angleCHAR);
  Serial.println(angle);

  
  if (angle < 30 && angle > -30) {
    desiredTheta = -angle;
  }

  //Time Reset
 

  //Reassign Angles
  newDegreeLeft = ((double)leftEnc.read() * ((360.0) / 3200.0));
  newDegreeRight = ((double)rightEnc.read() * ((360.0) / 3200.0));

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

  tanVelocityRight = ((1000 * DegreesToRadians * newDegreeRight - 1000 * DegreesToRadians * oldDegreeRight) / SampleTime) * WheelRadius;
  tanVelocityLeft = -((1000 * DegreesToRadians * newDegreeLeft - 1000 * DegreesToRadians * oldDegreeLeft) / SampleTime) * WheelRadius;
  errorTanVelocity = tanVelocityRight - tanVelocityLeft;
  double errorTanVelocityRight = errorTanVelocity;
  double errorTanVelocityLeft = errorTanVelocity;

  //button to reset the robot so it can try to find a new angle
  buttonState = digitalRead(11);
  //Serial.println(switchTime);
  if (buttonState == true) {
    switchTime = !switchTime;
    if (switchTime == true) {
      goTime = true; //flag to start moving towards an angle
      stateFindTape = 1;
      tapeNotFound = 1;
    }
    if (switchTime == false) {
      goTime = false;
      correctAngle = false;
      stateFindTape = 0; //<---- SET THIS TO 1 TO START THE SEQUENCE
      stateFixAngle = 0;
      stateMoveToStart = 0;
      stateFixAngle2 = 0;
      stateMoveToEnd = 0;
      analogWrite(PinSpeedR, 0);
      analogWrite(PinSpeedL, 0);
    }
    //resets encoders
    rightEnc.write(0);
    leftEnc.write(0);
    delay(200);
     
  }
  
if(goTime == true){
  //STATE 1: Finding Tape
  if (stateFindTape && tapeNotFound) {
    moveForward(0, tanVelocityRight, tanVelocityLeft, turnRateConverter(25));//turn in place
    Serial.println("state1");
  }
  if (angle > -30.0 && angle < 30.0) { //tape has been found
     moveForward(0, tanVelocityRight, tanVelocityLeft, turnRateConverter(15));//turning speed decreased
    tapeNotFound = false;
  }
  //Once tape is found, tapeNotFound should be 0
  if (stateFindTape && !tapeNotFound) {
     //stop rotating
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
    stateFindTape = 0;
    stateMoveToStart = 1; //setting flag to switch states
  }

if(stateMoveToStart && !waitedLongEnough){
  delay(State2WaitTime);
  waitedLongEnough = true;
}

  //STATE 2: Moving to tape
  if (stateMoveToStart == true && !commandToStop) {
   // moveForward(0.15, tanVelocityRight, tanVelocityLeft, 0);
    digitalWrite(PinDirectionR, LOW);
    digitalWrite(PinDirectionL, HIGH);
    analogWrite(PinSpeedR, 60);
    analogWrite(PinSpeedL, 63);//moves forward 0.15m/s with in a straight line (0 deg/s)
    if (angle == 777) { //we are close to the tape
      commandToStop = true;
    }
  }
  if (stateMoveToStart && commandToStop) { //#############
    delay(1000);
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);//stop rotating // will modify this based on testing so it actually reaches tape
    stateMoveToStart = 0;
    commandToStop = false; //reset flag
    stateFixAngle2 = 1;
    waitedLongEnough = 0;
  }
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

double turnRateConverter(double angle) {
  return WheelDistance * DegreesToRadians * angle;
}

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

//Multiply input angle by 0.9 for accuracy
bool fixAngle(double angle, double currentThetaRight, double currentThetaLeft, double errorTheta, double tanVelocityRight, double tanVelocityLeft) {

  double desiredTheta = angle;
  double deltaThetaRight = desiredTheta - currentThetaRight;
  double deltaThetaLeft = desiredTheta - currentThetaLeft;
  bool correctAngle = 0;

  static double integralRight = 0;
  static double integralLeft = 0;


  if (deltaThetaLeft > abs(closeEnough)) {
    moveForward(0, tanVelocityRight, tanVelocityLeft, (deltaThetaLeft * Kp + integralLeft * Ki));
  }
  if (deltaThetaLeft <= abs(closeEnough)) {
    analogWrite(PinSpeedR, 0);
    analogWrite(PinSpeedL, 0);
    correctAngle = true;
  }

  //Calculate integral right
  if (abs(deltaThetaRight) < intThreshhold) {
    integralRight += (double) abs(deltaThetaRight) * SampleTime / 1000.0;
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
    integralLeft += (double) abs(deltaThetaLeft) * SampleTime / 1000.0;
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



void moveForward(double desVelocity, double currentVelocityRight, double currentVelocityLeft, double desTurnRate) {
  static int analogRight = 0;
  static int analogLeft = 0;
  static double calculateRight = 0;
  static double calculateLeft = 0;

  double desVelocityRight = desVelocity + desTurnRate / 2;
  double desVelocityLeft = desVelocity - desTurnRate / 2;

  double deltaVelocityRight;
  deltaVelocityRight = desVelocityRight - currentVelocityRight;
  double deltaVelocityLeft;
  deltaVelocityLeft = desVelocityLeft - currentVelocityLeft;


  calculateRight += deltaVelocityRight * KpVel;
  calculateLeft += deltaVelocityLeft * KpVel;
  analogRight = calculateRight;
  analogLeft = calculateLeft;

  if (calculateRight > 255) {
    calculateRight = 255;
  }
  if (calculateRight < -255) {
    calculateRight = -255;
  }

  if (calculateLeft > 255) {
    calculateLeft = 255;
  }
  if (calculateLeft < -255) {
    calculateLeft = -255;
  }

  if (analogRight > 255) {
    analogRight = 255;
  }
  if (analogRight < -255) {
    analogRight = -255;
  }

  if (analogLeft > 255) {
    analogLeft = 255;
  }
  if (analogLeft < -255) {
    analogLeft = -255;
  }

  if (analogRight >= 0) {
    digitalWrite(PinDirectionR, LOW);
    Serial.println("Right high 1");
  }
  else if (analogRight < 0) {
    digitalWrite(PinDirectionR, HIGH);
    Serial.println("Right low 1");
  }
  if (analogLeft >= 0) {
    digitalWrite(PinDirectionL, HIGH);
    Serial.println("Left low 1");
  }
  else if (analogLeft < 0) {
    digitalWrite(PinDirectionL, LOW);
    Serial.println("Left high 1");
  }

//  if (analogRight <= 0 && (desVelocity - desTurnRate / 2 < 0)) {
//    digitalWrite(PinDirectionR, HIGH);
//    Serial.println("Right high");
//  }
//  else if (analogRight > 0 && (desVelocity - desTurnRate / 2 < 0)) {
//    digitalWrite(PinDirectionR, LOW);
//    Serial.println("Right low");
//  }
//  if (analogLeft <= 0 && (desVelocity - desTurnRate / 2 < 0)) {
//    digitalWrite(PinDirectionL, LOW);
//    Serial.println("Left low");
//  }
//  else if (analogLeft > 0 && (desVelocity - desTurnRate / 2 < 0)) {
//    digitalWrite(PinDirectionL, HIGH);
//    Serial.println("Left high");
//  }


  Serial.println(analogRight, 5);
  Serial.println(analogLeft, 5);
  //Serial.println(desVelocityRight, 5);

  analogWrite(PinSpeedR, abs(analogRight));
  analogWrite(PinSpeedL, abs(analogLeft));


  //Serial.println(leftEnc.read());

}

//Communication with Pi
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
