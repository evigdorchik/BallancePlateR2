#include <Servo.h>
#include <TouchScreen.h>

Servo X;
int xNeutral = 1427; //Value to keep plate level in X
int xMax = 700;
int xMin = 2350;

Servo Y;
int yNeutral = 1550; //Value to keep plate level in Y
int yMax = 700;
int yMin = 2350;

#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 8   // can be a digital pin
#define XP 9   // can be a digital pin

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

float estimateX = 0;
float estimateY = 0;
float alpha = .2;
int errorX = 0;
int errorXOld = 0;
float dErrorX = 0;
float sumErrorX = 0;

int errorY = 0;
int errorYOld = 0;
float dErrorY = 0;
float sumErrorY = 0;

int targetX = 300;
int targetY = 400;

int posX = xNeutral;
int posY = yNeutral;

unsigned long Time1, Time2;

int sumThreash = 200;

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
  // put your setup code here, to run once:

  X.attach(5);
  Y.attach(6);

  Serial.begin(9600);

}

//PID logic
int PID(float P, float I, float D, float error, float sumError, float dError)
{
  int out = (P * error) + (I * sumError) + (D * dError);
  return out;
}


void loop() {
  // put your main code here, to run repeatedly:

  TSPoint p = ts.getPoint(); // reading x,y value of the ball

  if (p.x != 0) {

    Time1 = millis();
    float dT = Time1 - Time2; //needs to be float for control calculations

    /***** Using a complementary filter for now but should improve ****/
    if (dT < 20) {
      estimateX = (alpha * estimateX) +((1-alpha) * float(p.x));
      estimateY = (alpha * estimateY) +((1-alpha) * float(p.y));
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
        targetX = 500;
        targetY = 500;
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
      dErrorX = (errorX - errorXOld) / dT;
      errorXOld = errorX;

      errorY = targetY - estimateY;
      dErrorY = (errorY - errorYOld) / dT;
      errorYOld = errorY;

      if (sumErrorX < sumThreash) { //antiwindup
        sumErrorX += errorX;
      }
      if (sumErrorY < sumThreash) {
        sumErrorY += errorY;
      }

      //Printing stuff for debugging (change format to csv file?)
      //  Serial.print(p.x);
      //  Serial.print("\t");
      //  Serial.println(p.y);
    }

    if (p.x == 0)
    {
      //X.writeMicroseconds(xNeutral);
      //Y.writeMicroseconds(yNeutral);
      X.writeMicroseconds(posX);
      Y.writeMicroseconds(posY);
      //Serial.println("here");
    }
    else
    {
      posX = xNeutral + PID(.45, .02, 185., errorX, sumErrorX, dErrorX);
      posY = yNeutral - PID(.455, .02, 185., errorY, sumErrorY, dErrorY);
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
      Serial.print(Time1);
      Serial.print(",");
      Serial.print(estimateX);
      Serial.print(",");
      Serial.println(targetX);
      //Serial.print(",");
      //Serial.println(targetY);
    }

    //  Serial.print(p.x);
    //  Serial.print("\t");
    //  Serial.println(sin(PI/2));

    // delay(16); //Need to keep the update slower than servo update (50Hz)
  }
}
