#include "Adafruit_TSC2007.h"
#include <Servo.h>

Adafruit_TSC2007 touch; //Touch sensor TSC2007

Servo X; //KST X12-508
int xNeutral = 1510; //Value to keep plate level in X
int xMax = 1000;
int xMin = 1950;

Servo Y; //KST X12-508
int yNeutral = 1542; //Value to keep plate level in Y
int yMax = 1000;
int yMin = 1855;


float estimateX = 0;
float estimateY = 0;
float alpha = .08;
int errorX = 0;
int errorXOld = 0;
float dErrorX = 0;
float sumErrorX = 0;

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
  X.attach(5); // X axis servo, pin 5
  Y.attach(6); // Y axis servo, pin 6
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!touch.begin()) {
    Serial.println("Couldn't find touch controller");
    while (1) delay(10);
  }
  Serial.println("Found touch controller");
}

//PID logic
int PID(float P, float I, float D, float error, float sumError, float dError)
{
  int out = (P * error) + (I * sumError) + (D * dError);
  return out;
}


void loop() {
  // put your main code here, to run repeatedly:

  uint16_t x, y, z1, z2;
  // if (touch.read_touch(&x, &y, &z1, &z2)) {
  //   Serial.print("Touch point: (");
  //   Serial.print(x); Serial.print(", ");
  //   Serial.print(y); Serial.print(", ");
  //   Serial.print(z1); Serial.print(" / ");
  //   Serial.print(z2); Serial.println(")");
  // }
  if (touch.read_touch(&x, &y, &z1, &z2)) { // Read a touch, begin controlling

    Time1 = millis();
    float dT = Time1 - Time2; //needs to be float for control calculations

    /***** Using a complementary filter for now but should improve ****/
    if (dT < 18) {
      // estimateX = (alpha * estimateX) +((1-alpha) * float(x));
      // estimateY = (alpha * estimateY) +((1-alpha) * float(y));
      estimateX = x;
      estimateY = y;
    }
    else {

      Time2 = Time1;

      /***** This is to make the ball go in different routes ******/
      if (inf) {

        if ( rightSide ) {
          targetX = 675 - (110 * cos(theta * DEG_TO_RAD));
          targetY = 500 + (200 * sin(theta * DEG_TO_RAD));
        }
        else {
          targetX = 325 + (110 * cos(theta * DEG_TO_RAD));
          targetY = 500 + (200 * sin(theta * DEG_TO_RAD));
        }
      }
      else if (circ) {
        targetX = 500 + (110 * cos(theta * DEG_TO_RAD));
        targetY = 500 + (200 * sin(theta * DEG_TO_RAD));
      }
      else if (cent) {
        targetX = 2000;
        targetY = 2000;
      }
      else if (hop) {
        if (CT > 150) {
          place = !place;
          CT = 0;
        }
        if (place) {
          targetX = 300;
          targetY = 500;
        }
        else {
          targetX = 700;
          targetY = 500;
        }
        CT++;
      }


      //*****Calculate errors for PID*****//
      errorX = targetX - estimateX;
      // dErrorX = (errorX - errorXOld) / dT;
      dErrorX = ((errorX - errorXOld) * alpha / dT) + ((1-alpha)*dErrorX); // complementary filter for D
      errorXOld = errorX;
      sumErrorX += errorX;
      // Antiwindup
      if (sumErrorX > sumThreash) {
        sumErrorX = sumThreash;
      }
      else if (sumErrorX < -1*sumThreash) {
        sumErrorX = -1* sumThreash;
      }

      errorY = targetY - estimateY;
      dErrorY = (errorY - errorYOld) / dT;
      dErrorY = ((errorY - errorYOld) * alpha / dT) + ((1-alpha)*dErrorY); // complementary filter for D
      errorYOld = errorY;
      sumErrorY += errorY;
      // Antiwindup
      if (sumErrorY > sumThreash) {
        sumErrorY = sumThreash;
      }
      else if (sumErrorY < -1*sumThreash) {
        sumErrorY = -1* sumThreash;
      }

      //Printing stuff for debugging (change format to csv file?)
      //  Serial.print(p.x);
      //  Serial.print("\t");
      //  Serial.println(p.y);
    }

    //***** Commanding the Servos *****//

    if (x == 0 || y == 0) // Don't move if getting unreasonable measurements
    {
      //X.writeMicroseconds(xNeutral);
      //Y.writeMicroseconds(yNeutral);
      X.writeMicroseconds(posX);
      Y.writeMicroseconds(posY);
      //Serial.println("here");
    }
    else
    {
      posX = xNeutral + PID(-0.11, -0.0003, -21, errorX, sumErrorX, dErrorX);
      posY = yNeutral - PID(0.11, 0.0003, 21, errorY, sumErrorY, dErrorY);
      /**Uncomment for debugging so platform doesn't move**/
      //      posX = xNeutral;
      //      posY = yNeutral;

      if (posX > xMin) {
        posX = xMin;
      }
      else if (posX < xMax) {
        posX = xMax;
      }
      X.writeMicroseconds(posX);

      if (posY > yMin) {
        posY = yMin;
      }
      else if (posY < yMax) {
        posY = yMax;
      }
      Y.writeMicroseconds(posY);
      //    Serial.print(posX);
      //    Serial.print("\t");
      //    Serial.println(posY);

      /**** This is how fast the ball moves in patterns ****/
      theta = theta + .6;
      if (theta >= 360) {
        rightSide = !rightSide;
        theta = 0;
      }

      //Tuing printout//
      // Serial.print("X_est:");
      // Serial.print(estimateX);
      // Serial.print(",");
      // Serial.print("X_target:");
      // Serial.print(targetX);
      // Serial.print(",");
      // Serial.print("X_err:");
      // Serial.print(errorX);
      // Serial.print(",");
      // Serial.print("X_derr:");
      // Serial.print(dErrorX * 100);
      // Serial.print(",");
      // Serial.print("X_interr:");
      // Serial.println(sumErrorX);

      // Serial.print("Y_est:");
      // Serial.print(estimateY);
      // Serial.print(",");
      // Serial.print("Y_target:");
      // Serial.print(targetY);
      // Serial.print(",");
      // Serial.print("Y_err:");
      // Serial.print(errorY);
      // Serial.print(",");
      // Serial.print("Y_derr:");
      // Serial.println(dErrorY);

      // Serial.print("dT:");
      // Serial.println(dT);
    }

    //  Serial.print(p.x);
    //  Serial.print("\t");
    //  Serial.println(sin(PI/2));

     //delay(8); //Need to keep the update slower than servo update (50Hz)
  }
}
