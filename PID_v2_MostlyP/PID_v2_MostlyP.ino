//VERSION 3 OF INCREMENTAL TESTING BY LAUREN and ISAAC
//Runs motors at speed based on sensor values

//currently:
//PROBLEMS:

// Initial PID loop control for line follower
#include <stdlib.h>

// Load motor shield libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


//Variables:
int lSense; // Set initial left sensor value
int rSense; // Set initial right sensor value
float Kp = 9; // Setpoint value for P control
int lRead; // Initialize variable for reading left sensor values
int rRead; // Initialize variable for reading right sensor values
int lSpeed; // Initialize variable for left motor speed
int rSpeed; // Intialize variable for right motor speed

// Pin setup
const byte lSensePin  = A0; //Initialize left sensor in analog pin 0
const byte rSensePin = A1; //Initialize right sensor in analog pin 1

// Call Adafruit library and set Adafruit motor values
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Initialize left motor on Motor port 4
Adafruit_DCMotor *lMotor  = AFMS.getMotor(4);
// Initialize right motor on Motor port 1
Adafruit_DCMotor *rMotor = AFMS.getMotor(1);


void setup() {
  // Set up Serial Port
  Serial.begin(9600);
  // Take initial sensor readings from left and right sensors
  lSense = analogRead(A0);
  rSense = analogRead(A1);
  // Set up Motors
  AFMS.begin();
  // Set initial speeds for left and right motors
  // From 0 to 255
  lMotor -> setSpeed(20);
  rMotor -> setSpeed(20);
  // Run Motors
  lMotor ->  run(FORWARD);
  rMotor ->  run(FORWARD);
  //
  lMotor -> run(RELEASE);
  rMotor -> run(RELEASE);
}

void loop() {
  //Get data from Sensors
  lRead = analogRead(lSensePin);
  rRead = analogRead(rSensePin);

  //Read incoming sensor data
  if (Serial.available() > 0) {

    Kp = Serial.parseFloat(); //Set new Kp value based on serial data from sensors
  }

  // Run Motors Forward
  lMotor -> run(FORWARD);
  rMotor -> run(FOWARD);

  //Equation for motor speeds based on sensor readings
  lSpeed = 20 - (lRead - lSense) / Kp // Sets speed based on sensor readings divided by setpoint value
  if (lSpeed < 0) {
    lSpeed = 0;
  }

  rSpeed = 20 - (rRead - rSense) / Kp; // Sets speed based on sensor readings divided by setpoint value

  if (rSpeed < 0) {
    rSpeed = 0;
  }

  //Set motor speeds based on distance readings
  lMotor -> setSpeed(lSpeed);
  rMotor -> setSpeed(rSpeed);

  // Print values for everything
  Serial.println(lSense);
  Serial.println(rSense);
  Serial.println(lRead);
  Serial.println(rRead);
  Serial.println(lSpeed);
  Serial.println(rSpeed);
  Serial.println(Kp);
}

