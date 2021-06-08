#include <Arduino.h>

uint8_t ledPin = 3;
uint8_t lightSensorPin = A1;

int ledHigh = 0;
int_fast16_t ledLow = 0;

int cutOffTrigger = 0;

int sensitivity = 3;

unsigned long startTime = 0;
unsigned long finishTime = 0;

double deltaTime = 0;

double averageLookback = 50;

double average = 0;




void setup()
{
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);

  pinMode(lightSensorPin, INPUT);

  Serial.println("Calibrating....");

  digitalWrite(ledPin, HIGH);
  delay(1000);
  ledHigh = analogRead(lightSensorPin);

  digitalWrite(ledPin, LOW);
  delay(1000);
  ledLow = analogRead(lightSensorPin);
  digitalWrite(ledPin, LOW);

  cutOffTrigger = (ledHigh + (sensitivity * ledLow)) / (sensitivity + 1);

  Serial.print("ledHigh: ");
  Serial.println((int)ledHigh);
  Serial.print("ledLow: ");
  Serial.println((int)ledLow);
  Serial.print("curOffTrigger: ");
  Serial.println((int)cutOffTrigger);

  Serial.println("beginning loop");
}


void runningAverage(double * average, double newValue){

  *average -= *average / averageLookback;
  *average += newValue / averageLookback;

}



void loop()
{

  digitalWrite(ledPin, HIGH);
  startTime = micros();

  while (analogRead(lightSensorPin) > cutOffTrigger)
  {
  }

  finishTime = micros();
  
  digitalWrite(ledPin, LOW);

  deltaTime = (double)finishTime - (double)startTime;


  runningAverage(&average, deltaTime);

  Serial.print("deltaTime: ");
  Serial.print(deltaTime / 1000, 4);
  Serial.print("ms  |   Running Average: ");
  Serial.print(average / 1000, 4);
  Serial.println("ms");

  delay(100);
}

