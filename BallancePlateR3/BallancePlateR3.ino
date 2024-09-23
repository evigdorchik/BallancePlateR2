// #include "Adafruit_TSC2007.h"
#include"TSC2007.h"
#include <Servo.h>

// Adafruit_TSC2007 touch; //Touch sensor TSC2007

TSC2007 touch;

Servo X; //KST X12-508
int xNeutral = 1510; //Value to keep plate level in X
int xMax = 1000;
int xMin = 1950;

Servo Y; //KST X12-508
int yNeutral = 1542; //Value to keep plate level in Y
int yMax = 1000;
int yMin = 1855;


// float estimateX = 0;
// float estimateY = 0;
// float alpha = .08;
// int errorX = 0;
// int errorXOld = 0;
// float dErrorX = 0;
// float sumErrorX = 0;

int errorY = 0;
int errorYOld = 0;
float dErrorY = 0;
float sumErrorY = 0;

int targetX = 2000;
int targetY = 2000;

int posX = xNeutral;
int posY = yNeutral;

unsigned long Time1, Time2;

int sumThreash = 1000;

bool rightSide = true;

float theta = 0;
#define DEG_TO_RAD 0.017453292519943295769236907684886

bool inf = false; //infinuty sign from 2 circles
bool circ = false; //circle around center of board
bool cent = true; //stay on center of board
bool hop = false; //hopping back and forth (mostly for tuning)

bool place = true; //used to change the place on hop
int CT = 0;

void setup() {
  Wire.begin();
  X.attach(5); // X axis servo, pin 5
  Y.attach(6); // Y axis servo, pin 6
  Serial.begin(115200);
  while (!Serial) delay(10);

  // if (!touch.begin()) {
  //   Serial.println("Couldn't find touch controller");
  //   while (1) delay(10);
  // }
  // Serial.println("Found touch controller");
}

//PID logic
int PID(float P, float I, float D, float error, float sumError, float dError)
{
  int out = (P * error) + (I * sumError) + (D * dError);
  return out;
}


void loop() {
  // put your main code here, to run repeatedly:

  Time1 = millis();
  float dT = Time1 - Time2; //needs to be float for control calculations

  Time2 = Time1;

  X.writeMicroseconds(1500);
  Y.writeMicroseconds(1500);

}
