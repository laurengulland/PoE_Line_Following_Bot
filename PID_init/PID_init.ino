// Initial PID loop control for line follower

#include <stdlib.h>

// Load motor shield libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <PID_v1.h>

// Controlling constants

const int loopduration = 10; //(ms) This is the inverse of the main loop frequency

const int FORWARD_POWER = 20; // 0...255
const int TURN_POWER = 20; // 0...255


// Pin setup
const byte leftSensorPin  = A1;
const byte rightSensorPin = A0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Global variable setup (things that change each loop)
long lastActionTime;

// Setup PID controller
double PIDerror=0, PIDsetpoint=0, PIDoutput;
double kp=1,ki=0,kd=0;

PID pid(&PIDerror, &PIDoutput, &PIDsetpoint, kp, ki, kd, DIRECT);

void setup()
{
  Serial.begin(9600);

  lastActionTime = millis();

  // Initialize pins
  // Note that the analog sensors don't need initialization
  AFMS.begin();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(loopduration);
  pid.SetOutputLimits(-1, 1);
}

long Left = 0;
long Right = 0;
int count = 0;

void loop()
{
  handleIncomingSerial();

  int lRead = 0;
  int rRead = 0;
  getMeasurements(&lRead, &rRead);

  Left += lRead;
  Right += rRead;
  count++;

  // Takes average of two readings
  long time = millis();
  if (time - lastActionTime > loopduration) {
    float lAvg = float(Left) / count;
    float rAvg = float(Right) / count;

    Serial.print(lineOffset(lAvg, rAvg));
    Serial.print("\t");
    Serial.println(PIDoutput);

    lineFollowPid(lAvg, rAvg);   

    // Reset counting variables
    Right = Left = 0;
    count = 0;

    // Ensures average loop duration is loopduration
    lastActionTime = lastActionTime + loopduration*int((time-lastActionTime) / loopduration);
  }
}

// Implements control of motors.
void lineFollowPid(float lAvg, float rAvg)
{
  float error = lineOffset(lAvg, rAvg);
  PIDerror = error;

  pid.Compute();

  // Decide whether to go left or right
  //Right = positive
  //Left = negative
  float turnFactor = PIDoutput;

  int lPower = FORWARD_POWER + turnFactor * TURN_POWER;
  int rPower  = FORWARD_POWER - turnFactor * TURN_POWER;

  normalizePowers(&lPower, &rPower, 255);

  driveMotors(lPower, rPower);
}

// ensures that motor speeds are within limits while maintaining ratio between motors
void normalizePowers(int *left, int *right, int limit){
  int maxabs = max(abs(*left), abs(*right));
  if(maxabs > limit)
  {
      *left = *left * (limit / maxabs);
      *right = *right * (limit / maxabs);
  }
}

// Positive values = Robot right off the line
// Negative values = Robot left off the line 
// Inputs are raw sensor values (0-1023)
float lineOffset(float lAvg, float rAvg)
{
  return map(lAvg, 780, 880, -100, 100) / 100.0;
  //return rAvg - lAvg;
}

void handleIncomingSerial()
{
  if(Serial.available() > 0){

    Serial.setTimeout(100);
    // Read first number from serial stream
    kp = Serial.parseFloat();
    Serial.read();
    ki = Serial.parseFloat();
    Serial.read();
    kd = Serial.parseFloat();
    Serial.read();

    // Check to make sure nothing went wrong
    while(Serial.available()){
        Serial.read();
    }

    pid.SetTunings(kp, ki, kd);
    writeSerial();
  }
}

void writeSerial()
{
  Serial.println("Tunings set to (kp, ki, kd) = ");
  Serial.print("\t(");
  Serial.print(kp);
  Serial.print(", ");
  Serial.print(ki);
  Serial.print(", ");
  Serial.print(kd);
  Serial.print(")\n");
}

void driveMotors(int lPower, int rPower){
  // Inputs leftPower and rightPower vary from -255...255
  // Code: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors

  // For each motor, decide whether to run it FORWARD, BACKWARD, (or RELEASE)
  // These are ternary operators, returning FORWARD if power > 0 and backward otherwise.
  byte lDirection  = (lPower  > 0) ? FORWARD : BACKWARD;
  byte rDirection = (rPower > 0) ? FORWARD : BACKWARD;
  
  // Set motor speeds
  lMotor ->  setSpeed(abs(lPower));
  rMotor  ->  setSpeed(abs(rPower));

  // Set motor directions
  lMotor ->  run(lDirection);
  rMotor  ->  run(rDirection);

}

void getMeasurements(int *lRead, int *rRead)
{
  *lRead = analogRead(leftSensorPin);
  *rRead = analogRead(rightSensorPin);
}
