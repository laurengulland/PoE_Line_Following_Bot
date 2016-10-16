//VERSION 2 OF INCREMENTAL TESTING BY LAUREN
//runs motors at constant speed, prints IR data to serial

//currently: runs both motors successfully and also prints data from both sensors to serial
//PROBLEMS: all data from sensors is 1023. need to debug.

// Initial PID loop control for line follower
#include <stdlib.h>

// Load motor shield libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <PID_v1.h>

//Set up sensor things
const int averagingDuration = 200; //Pick duration for average; AFFECTS PERFORMANCE OF BOT
long lastActionTime;
long Left = 0;
long Right = 0;
int count = 0;

// Pin setup
const byte leftSensorPin  = A0; //Initialize left sensor in pin1
const byte rightSensorPin = A1; //Initialize right sensor in pin2

// Call Adafruit library and set Adafruit motor values
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *lMotor  = AFMS.getMotor(4);
Adafruit_DCMotor *rMotor = AFMS.getMotor(1);


void setup() {
  // Set up Serial Port
  Serial.begin(9600);
  //Set up Motors
  AFMS.begin();
  lMotor -> setSpeed(20);
  rMotor -> setSpeed(20);
  //Initial millis start
  lastActionTime = millis();

}

void loop() {
  // Run Motors
  lMotor ->  run(FORWARD);
  rMotor ->  run(FORWARD);
  
  //Get data from Sensors
  int lRead = analogRead(leftSensorPin);
  int rRead = analogRead(rightSensorPin);

  Left += lRead;
  Right += rRead;
  count++;

  // Take average of two sensor readings
  long time = millis();
  if (time - lastActionTime > averagingDuration){
    float lAvg = float(Left) / count;
    float rAvg = float(Right) / count;

    writeSerial(lAvg, rAvg);

    lastActionTime = time;
    Right = Left = 0;
    count = 0;

  }
}

void writeSerial(float lAvg, float rAvg)
{
  Serial.print(lAvg);
  Serial.print(",\t");
  Serial.print(rAvg);
  Serial.print("\n");
}

