#include <Servo.h>
int const servoPin = 5;
Servo servo;

int const MAX_ANGLE = 180;
int const MIN_ANGLE = 0;
int const NO_ANGLE = 90;


#define DEBUG
#ifdef DEBUG
double courseTo = 34;
int currentHeading = 8;
#endif


void setup()
{
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin, 620, 2400);//, 1060, 1920);
  digitalWrite(servoPin, LOW);  
  setDirection(NO_ANGLE);
  
}


void loop()
{
 // servo.writeMicroseconds(1500);
  //return;
  
  
  double relativeHeading = (courseTo - currentHeading);//fakeAngle);
  double relativeHeadingClamped = relativeHeading;
  if(relativeHeadingClamped < -90) {
    relativeHeadingClamped = -90;
  }
  if(relativeHeadingClamped > 90) {
    relativeHeadingClamped = 90;
  }
  double angle = NO_ANGLE + relativeHeadingClamped;//relativeHeading + 90;
  //}     
  setDirection(angle);
  Serial.print("relativeHeading = ");
  Serial.println(relativeHeading);
  Serial.print("relativeHeadingClamped = ");
  Serial.println(relativeHeadingClamped);
  Serial.print("Servo angle is : ");
  Serial.println(angle);
  
}


/**
 * @Description: Anything to do with controlling the servo.
 * @param angle int[in] the angle the rudder should be set to
**/
void setDirection(double angle)//double courseTo, int currentHeading, double distanceKm)
{
  int angleMicroSec = angle * 10 + 600; // angle should be between 0 and 180
  servo.writeMicroseconds(angleMicroSec); // 600 to 2400  
  servo.write(angle); 
}
