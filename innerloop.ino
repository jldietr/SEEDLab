/* Arduino
   control speed and angular speed
*/

// libraries
#include <Encoder.h>
#include <math.h>
#include <Wire.h> //connects Pi to Arduino for LCD screen
#include <stdlib.h>

#define SLAVE_ADDRESS 0x04
char angleCHAR[6];
int angleRAW[6];
int zeroTrack;
string angleName;

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
#define RadiansToDegrees PI/180

#define intThreshhold 5
#define closeEnough 0.5


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

double angle = 20; //TEST ANGLE

double Ke = 0.01;
double Kp = 6;
double Ki = 1.3;


void setup() {

    // serial communication initialization
    Serial.begin(57600);

    // assigns pins as either inputs or outputs
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

    /* Pi connection
      Wire.begin(SLAVE_ADDRESS); //sends data to LCD
      Wire.onRequest(sendData);
      Wire.onReceive(receiveData);
      -*/
} // end of setup


double distanceRightWheelTravelled = 0;
double distanceLeftWheelTravelled = 0;
double currentTheta = 0;
double integral = 0;
int analog = 0;
int deltaTheta = 0;
unsigned long currentTime = 0;


void loop() {

    // static variables

    // static double newDegreeLeft = 0;
    //static double newDegreeRight = 0;
    //static double oldDegreeRight = 0;
    // static double oldDegreeLeft = 0;






    // measures time for delay
    currentTime = millis();

    // take sample of position,calculate position, calculate speed




    distanceRightWheelTravelled =
            ((double) rightEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)
    distanceLeftWheelTravelled =
            ((double) leftEnc.read() * ((2 * PI) / 3200)) * WheelRadius; // X_wheel(t) = Radius_wheel * Theta_wheel(t)

    errorPosition = distanceRightWheelTravelled - distanceLeftWheelTravelled;

    currentTheta = -RadiansToDegrees * 3200 *
                   (distanceRightWheelTravelled / (WheelDistance * 0.5)); // Theta_sys(t) = X_wheel(t) / Radius_sys




    // calculate angular velocity of wheels
    // velocityRight = (1000 * (newDegreeRight - oldDegreeRight)) / SampleTime;
    //velocityLeft = (1000 * (newDegreeLeft - oldDegreeLeft)) / SampleTime;

    // calculate speed of system
    //actualXY = WheelRadius * 0.5 * (newDegreeRight + newDegreeLeft) * DegreesToRadians;
    //actualRotation = (WheelRadius / WheelDistance) * (newDegreeLeft - newDegreeRight);

    // get desired angle by converting array of chars to float
    angle = atof(angleName.c_str());
    desiredTheta = angle; //TEST ANGLE, DON'T USE FOR ACTUAL
    deltaTheta = currentTheta - desiredTheta;

    //determine voltage
    if (deltaTheta != 0) {
        analog = Kp * (double) abs(deltaTheta) + integral * Ki;
        if (analog > 255) {
            analog = 255;
        }
        if (analog < 0) {
            analog = 0;
        }
        if (errorPosition < 0) {
            analogWrite(PinSpeedR, analog);
            analogWrite(PinSpeedL, analog);
        }
        if (errorPosition >= 0) {
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
    }

    if (abs(deltaTheta) < intThreshhold) {
        integral += (double) abs(deltaTheta) * 0.04;
    } else if (abs(deltaTheta) > intThreshhold) {
        integral = 0;
    }
    if (abs(deltaTheta) < closeEnough) {
        integral = 0;
    }
    Serial.println(currentTheta);
    //Serial.println(leftEnc.read());
    // reassign angles

    // ensures function isn't taking too long
    if (millis() > (currentTime + SampleTime)) Serial.println("ERROR: Under Sampling!");

    // creates delay of SampleTime ms
    while (millis() < (currentTime + SampleTime));

} // end of loop

void receiveData(int byteCount) {

    while (Wire.available()) {
        angle = Wire.read();
        if (angle != 0) {
            Serial.print("data received: ");
            Serial.println(angle);
            angleRAW[i] = angle;
            angleCHAR[i] = angleRAW[i];
            angleName += angleCHAR[i];
            i++; // need to reset i once we have the angle
            zeroTrack = 0;
            if (i == 6) {
                i = 0;
            }
        }
        if (angle == 0) {
            zeroTrack++;
            if (zeroTrack == 2) {
                break;
            }
        }
    }
}
