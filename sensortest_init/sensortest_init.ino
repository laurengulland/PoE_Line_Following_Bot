#include <stdlib.h>

const int averagingDuration = 200; //Pick duration for average; AFFECTS PERFORMANCE OF BOT

const byte SensorPin1 = A1;
const byte SensorPin2 = A0;
// const byte leftMotorPin = 9;
// const byte rightMotorPin = 10;

long lastActionTime;

void setup()
{
	Serial.begin(9600);

	lastActionTime = millis();

	// Initialize pins
	// analog sensors don't need initialization
}

long Left = 0;
long Right = 0;
int count = 0;

void loop()
{
	int lRead = 0;
	int rRead = 0;
	getMeasurements(&lRead, &rRead);

	Left += lRead;
	Right += rRead;
	count++;

	// Take average of two readings
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

void getMeasurements(int *lRead, int *rRead)
{
	*lRead = analogRead(SensorPin1);
	*rRead = analogRead(SensorPin2);
}
