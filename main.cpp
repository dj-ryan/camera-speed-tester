#include <Arduino.h>

uint8_t ledPin = 3;
uint8_t lightSensorPin = A1;

int ledHigh = 0;
int ledLow = 0;

int cutOffTrigger = 0;

int threshold = 100;

unsigned long startTime = 0;
unsigned long finishTime = 0;

double deltaTime = 0;

double averageLookback = 10;

double average = 0;




void setup()
{
  // put your setup code here, to run once:
  bool isCalibrated = false;
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);

  while(!isCalibrated) {
    Serial.println("Calibrating....");

    digitalWrite(ledPin, HIGH);
    delay(1000);
    ledHigh = analogRead(lightSensorPin);
    digitalWrite(ledPin, LOW);
    delay(1000);
    ledLow = analogRead(lightSensorPin);
    digitalWrite(ledPin, LOW);

    if(ledLow - ledHigh >= 300) {
      cutOffTrigger = ledHigh + threshold;
      isCalibrated = true;
      Serial.println("Calibration complete...");
    } else {
      Serial.println("Calibration failed. Trying again...");
    }
    delay(1000);
    // cutOffTrigger = (ledHigh + ledLow) / 2;
  }

  Serial.print("ledHigh:\t");
  Serial.println((int)ledHigh);
  Serial.print("ledLow:\t");
  Serial.println((int)ledLow);
  Serial.print("cutOffTrigger:\t");
  Serial.println((int)cutOffTrigger);
  delay(1000)
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

  while (analogRead(lightSensorPin) > cutOffTrigger){}

  finishTime = micros();  
  digitalWrite(ledPin, LOW);
  deltaTime = (double)finishTime - (double)startTime;

  runningAverage(&average, deltaTime);

  Serial.print("deltaTime:\t");
  Serial.print(deltaTime / 1000, 4);
  Serial.print("ms  |  Running Average:\t");
  Serial.print(average / 1000, 4);
  Serial.println("ms");

  delay(300);
}

