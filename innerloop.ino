/* Arduino
*  control speed and angular speed
*/
 
// libraries
#include <Encoder.h>

// system constants
#define SampleTime      40    // sampling time in milliseconds
#define MaxVoltage      7.5   // maximum voltage of the input into the motor
#define WheelRadius     0.0766   // radius of wheel in meters
#define WheelDistance      // distance between wheels in meters

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

void setup() {

  // serial communication initialization
  Serial.begin(9600);

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

  //variables
  
    // measures time for delay
  currentTime = millis();
  //take sample of position,calculate position, calculate speed
  //determine voltage
  
 // ensures function isn't taking too long
  if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SampleTime ms
  while(millis() < (currentTime + SampleTime));
  
} // end of loop
