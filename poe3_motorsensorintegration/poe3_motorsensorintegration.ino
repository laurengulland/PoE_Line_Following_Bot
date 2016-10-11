// Has not been updated yet
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
// #include <utility/Adafruit_PWMServoDriver.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

Servo left;
Servo right;

int sense1 = 7;
int sense2 = 8;

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Servos.attach(9, 800, 2200); // Left
  Servos.attach(10, 800, 2200); // Right
  motor1->setSpeed(100); 
  motor2->setSpeed(100); 
}

void loop() {
  // put your main code here, to run repeatedly:
  motor1->run(FORWARD);
  motor2->run(FORWARD);

}
