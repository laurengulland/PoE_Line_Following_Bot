#include <PololuQTRSensors.h>
#include <Servo.h>
#include <MegaServo.h>
#define NBR_SERVOS 3
#define FIRST_SERVO_PIN 2

Servo left;
Servo right;
Servo tower;

MegaServo Servos[NBR_SERVOS] ;

int pingPin = 7;
int mid = 0;
int mn = 0;
int mx = 0;

void setup()
{

  Servos[0].attach(2, 800, 2200); //tower
  Servos[1].attach(9, 800, 2200); //left
  Servos[2].attach(10, 800, 2200); //right
  Serial.begin(9600);
  Servos[0].write(65);
  digitalWrite(13, LOW);

  Servos[2].write(90);
  Servos[1].write(90);

  for (int i = 0; i < 5000; i++)
  {
    digitalWrite(13, HIGH);

    int val = 0;
    for (int j = 0; j <= 5; j++)
    {
      val = analogRead(j);
      if (val >= mx)
        mx = val;
      if (val <= mn)
        mn = val;
    }
    delay(1);
  }

  mid = ((mx + mn) / 2);
  digitalWrite(13, LOW);

  Servos[2].write(90);
  Servos[1].write(90);
}
void loop()
{

  int s0 = 0;
  int s1 = 0;
  int s2 = 0;
  int s3 = 0;
  int s4 = 0;
  int s5 = 0;

  s0 = analogRead(0);
  s1 = analogRead(1);
  s2 = analogRead(2);
  s3 = analogRead(3);
  s4 = analogRead(4);
  s5 = analogRead(5);

  Serial.print("Mid: ");
  Serial.print(mid);
  Serial.print(" ");
  Serial.print(s0);
  Serial.print(" ");
  Serial.print(s1);
  Serial.print(" ");
  Serial.print(s2);
  Serial.print(" ");
  Serial.print(s3);
  Serial.print(" ");
  Serial.print(s4);
  Serial.print(" ");
  Serial.print(s5);
  Serial.print(" ");
  Serial.println();

  Servos[2].write(180);
  Servos[1].write(0);

  delay(10);

  if ((((s0 + s1 + s2) / 3) > (((s3 + s4 + s5) / 3) + 250)) && (!((s0 > mid) && (s5 > mid))))
  {
    Servos[2].write(180);
    Servos[1].write(90);
    Serial.print(" RIGHT");
    delay(abs((((s5 + s4 + s3) / 3) - ((s0 + s1 + s2) / 3)) / 2));
  }
  if ((((s0 + s1 + s2) / 3) < (((s3 + s4 + s5) / 3) - 250)) && (!((s0 > mid) && (s5 > mid))))
  {
    Servos[2].write(90);
    Servos[1].write(0);
    Serial.print(" LEFT");
    delay(abs((((s5 + s4 + s3) / 3) - ((s0 + s1 + s2) / 3)) / 2));
  }
  if ((s0 > mid) && (s5 > mid))
  {
    Servos[2].write(90);
    Servos[1].write(90);
    Serial.print(" STOP");
    if ((s0 > mid) && (s5 > mid))
    {
      Servos[2].write(90);
      Servos[1].write(90);
      Serial.print(" STOP");
      if ((s0 > mid) && (s5 > mid))
      {
        Servos[2].write(90);
        Servos[1].write(90);
        Serial.print(" STOP");
        for (int k = 0; k < 50; k++)
        {
          digitalWrite(13, HIGH);
          delay(100);
          digitalWrite(13, LOW);
          delay(100);
        }
        delay(5000);
      }
    }
  }

}
