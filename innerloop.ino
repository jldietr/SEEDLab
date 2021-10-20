/* Arduino
*  control speed and angular speed
*/
 
// libraries
#include <Encoder.h>

// system constants
#define SampleTime      40    // sampling time in milliseconds
#define MaxVoltage      7.5   // maximum voltage of the input into the motor
#define WheelRadius     0.0766   // radius of wheel in meters
//TODO: Measure wheelbase for now assume 1/3 of a meter
#define WheelDistance   0.3   // distance between wheels in meters

// conversion constants
#define DegreeInRadians     0.01745329
#define MeterInFeet  	    0.3048 

// pins
#define PinChannelRA      2     // encoder input pin for right motor channel A
#define PinChannelLA      3     // encoder input pin for left motor channel A
#define PinChannelRB      5     // encoder input pin for right motor channel B
#define PinChannelLB      6     // encoder input pin for left motor channel B
#define Enable		  4	//enable motor
#define PinDirectionR     7     // right motor direction
#define PinDirectionL     8     // left motor direction
#define PinSpeedR         9    // PWM pin for right motor speed
#define PinSpeedL         10    // PWM pin for left motor speed    

// sets encoder functions
Encoder rightEnc(PinChannelRA, PinChannelRB);
Encoder leftEnc(PinChannelLA, PinChannelLB); 

// global variables
/*
actual vs desired + error
position distnace, position rotation
speed distance, speed rotation
*/
double actualXY = 0; //current position
double actualTheta = 0; //current direction facing
double actualSpeed = 0; //current speed
double actualRotation = 0; //current change in angle
double desiredXY = 0;
double desiredTheta = 0;
//double desiredSpeed = 0;
//double desiredRotation = 0;
//double errorXY = 0;
//double errorTheta = 0;
//double errorSpeed = 0;
//double errorRotation = 0;


void setup() {

  // serial communication initialization
  Serial.begin(57600);

  // assigns pins as either inputs or outputs
  pinMode(PinChannelRA, INPUT);
  pinMode(PinChannelLA, INPUT);
  pinMode(PinChannelRB, INPUT);
  pinMode(PinChannelLB, INPUT);
  pinMode(Enable, OUTPUT);
  pinMode(PinDirectionR, OUTPUT);
  pinMode(PinDirectionL, OUTPUT);
  pinMode(PinSpeedR, OUTPUT);
  pinMode(PinSpeedL, OUTPUT);

  // enables motors
  digitalWrite(Enable, HIGH);

  // writes direction to motor
  digitalWrite(PinDirectionR, LOW);
  digitalWrite(PinDirectionL, LOW);

} // end of setup

void loop() {

  // variables
 double leftWheelRotation = 0;
 double rightWheelRotation = 0;
  // measures time for delay
 currentTime = millis();
  //take sample of position,calculate position, calculate speed
 desiredXY = (WheelRadius) * ;
 desiredRotation = (WheelRadius/WheelDistance) * (leftWheelRotation - rightWheelRotation);
  //determine voltage
   
 // ensures function isn't taking too long
  if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SampleTime ms
  while(millis() < (currentTime + SampleTime));
  
} // end of loop
