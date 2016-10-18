//PoE Lab 3: Line Following Robot
//Lauren Gulland and Isaac Vandor
//VERSION 3 OF INCREMENTAL TESTING PID Control
//Runs motors at speed based on user-inputted threshold value and setpoint

//currently: Works Jankily
//PROBLEMS: Needs more Tuning (Success is heavily dependent upon position of sensors at start)

// Include Libraries
// Initial PID loop control for line follower
#include <stdlib.h>
// Load motor shield libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


// Initialize Variables
int lSense; // Set initial left sensor value (Calibration off-line)
int rSense; // Set initial right sensor value (Calibration off-line)
float Kp = 2; // Setpoint value for "P" control (USER INPUTTED)
int lRead; // Initialize variable for reading left sensor values (Changes based on line readings)
int rRead; // Initialize variable for reading right sensor values (Changes based on line readings)
int lSpeed; // Initialize variable for left motor speed (Changes based on controller loop)
int rSpeed; // Intialize variable for right motor speed (Changes based on controller loop)

// Sensor Pin Setup
const byte lSensePin  = A0; //Initialize left sensor in analog pin 0
const byte rSensePin = A1; //Initialize right sensor in analog pin 1

// Call Adafruit library and set Adafruit motor values
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Initialize left motor on Motor port 4
Adafruit_DCMotor *lMotor  = AFMS.getMotor(4);
// Initialize right motor on Motor port 1
Adafruit_DCMotor *rMotor = AFMS.getMotor(1);


void setup() {
  // Start Serial Communications
  Serial.begin(9600);
  // Take initial sensor readings from left and right sensors
  lSense = analogRead(lSensePin);
  rSense = analogRead(rSensePin);
  // Start Adafruit Motor Shield setup
  AFMS.begin();
  // Set initial speeds for left and right motors
  // From 0 to 255
  // Right Motor must be set slightly faster than left for this chassis setup (DON'T FORGET THIS)
  lMotor -> setSpeed(30);
  rMotor -> setSpeed(35);
  // START YOUR ENGINES
  lMotor ->  run(FORWARD);
  rMotor ->  run(FORWARD);
}

void loop() {
  //Get data from Sensors
  lRead = analogRead(lSensePin);
  rRead = analogRead(rSensePin);

  // Run Motors Forward
  lMotor -> run(FORWARD);
  rMotor -> run(FORWARD);

  //Equation for motor speeds based on sensor readings
  // lSense - lRead = Error from line
  // Kp = Setpoint Value
  // As our sensor data varies only by a small output, we decided to implement a threshold value and divide by our setpoint value
  lSpeed = 30 - (lSense - lRead) / Kp; // Sets Left motor speed based on sensor readings divided by setpoint value
  rSpeed = 30 - (rSense - rRead) / Kp; // Sets Right speed based on sensor readings divided by setpoint value

  //Set motor speeds based on P controller above
  lMotor -> setSpeed(lSpeed);
  rMotor -> setSpeed(rSpeed);

  // Print values for everything
  //Useful for testing
  Serial.println(lSense);
  Serial.println(rSense);
  Serial.println(lRead);
  Serial.println(rRead);
  Serial.println(lSpeed);
  Serial.println(rSpeed);
  Serial.println(Kp);
}

