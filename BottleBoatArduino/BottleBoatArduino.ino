#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
#include "TinyGPS.h" // Downloaded from : https://github.com/mikalhart/TinyGPS/releases/tag/v13

Servo servo;
TinyGPSPlus gps; // We are using a EM-408 GPS.  Also see http://arduiniana.org/libraries/tinygps/

SoftwareSerial ss(4,3); // For the GPS. 4,3 are the pins that the GPS are attached too.

//list of arduino pins the devices are conected to. 
int const motorPin = 9;
int const servoPin = 10;

int const MAX_ANGLE = 270;
int const MIN_ANGLE = 100;
int const NO_ANGLE = 180;

double waypoints[][2] = { {1,2}, {3,4} }; //{ {latitude, longitude}, .... } the points we want the boat to sail too.
int currentWaypointIndex = 0;

/**
 *
**/
void setup()
{
  Serial.begin(9600);
  
 //setup servo/rudder 
 servo.attach(servoPin);
 
 //setup motor 
 pinMode(motorPin, OUTPUT);
 digitalWrite(motorPin, LOW);
 
 //setup compass
 init_compass();
 
 //setup GPS
 //TODO 
}


/**
 * @Description: The main part of the program
**/
void loop()
{
  //Serial.println(get_heading()); 
  
  if(ss.available() > 0)
  {
    gps.encode(ss.read()); // at the start of each loop we want to get what the GPS coordinates 
    
    double waypointLat = waypoints[currentWaypointIndex][0];
    double waypointLng = waypoints[currentWaypointIndex][1];
    
    // check if we have already reached the waypoint
    double distanceKm = gps.distanceBetween(gps.location.lat(), gps.location.lng(), waypointLat, waypointLng) / 1000.0;    
    if (distanceKm < 0.014)// we will try to get within 14meters of the target.
    {
      currentWaypointIndex++;   
      // will error once we have finished ;)
      waypointLat = waypoints[currentWaypointIndex][0];
      waypointLng = waypoints[currentWaypointIndex][1];     
    }
     
    
    double courseTo = gps.courseTo(gps.location.lat(),gps.location.lng(), waypointLat, waypointLng);    
    int currentHeading = get_heading();
    
    // set the angle of the servo, if we are heading in the wrong direction
    while ( checkHeading(courseTo, currentHeading) )
    {      
      //commented out test code
     // double currentHeading = 0;
     // double courseTo = 180;
        
      double angleDiff = courseTo - currentHeading;
      int angle = sq(angleDiff)/405; // 405 becuase we never want it to go above 80 (don't wnat your rudder back-to-front or flat against boat)
      angle += 5; // just to make sure you are actually going to be turning. 
      if ((angleDiff < 0) && (angleDiff > -180))//turn anticlockwise
      {        
        angle = NO_ANGLE + angle
      }
      else //turn clockwise
      {
        angle = NO_ANGLE - angle 
      }     
      setDirection(angle);
   
        
      setMotorSpeed(100); 
      
      courseTo = gps.courseTo(gps.location.lat(),gps.location.lng(), waypointLat, waypointLng);    
      currentHeading = get_heading();
      
      delay(100);
    }   
    
    
    // when we get to within 4meters of the waypoint, slow down a little.
    if (distanceKm < 0.004)
    {
      setMotorSpeed(100); 
    }
    else
    {
      setMotorSpeed(270); 
    }      
    
  }
}

/**
*
**/
boolean checkHeading(double courseTo, int currentHeading)
{
  return (abs(courseTo - currentHeading) < 5);  
}


/**
 * @Description: Anything to do with controlling the servo.
 * @param angle int[in] the angle the rudder should be set to
**/
void setDirection(int angle)//double courseTo, int currentHeading, double distanceKm)
{
  int actualAngle = map(angle, 0, 1023, 0, 179); //see : http://arduino.cc/en/reference/map 
  servo.write(actualAngle); 
}

/**
 * @Description: Anything to do with controlling the motor
 *
**/
void setMotorSpeed(int motorSpeed)
{
  analogWrite(motorPin, motorSpeed);
}



//--------------------------------------------------//
//  The Compass                                     //
//--------------------------------------------------//
#define HMC5883_ADDRESS				0x1E
#define HMC5883_REG_DATA			0x03
#define HMC5883_REG_WRITE_MODE			0x02

/**
 * Initialises the compass by first starting up I2C and then setting the compass's mode.
 */
void init_compass()
{
	// starts i2c
	Wire.begin();
	Serial.println("Started I2C");
	delay(50);

	// change the compass to continuous 
        //Put the HMC5883 IC into the correct operating mode
        Wire.beginTransmission(HMC5883_ADDRESS); //open communication with HMC5883
        Wire.write(HMC5883_REG_WRITE_MODE); //select mode register
        Wire.write(0x00); //continuous measurement mode
        Wire.endTransmission();
	 
	Serial.println("HMC5883 set to continous mode");
	delay(50);
}

/**
 * Returns the forward heading of the compass
 */
int get_heading()
{
        //Serial.println("get_heading");
	byte x, y, z;
	double heading;

	// set the register on the device we want to read.
	Wire.beginTransmission(HMC5883_ADDRESS);
	Wire.write(HMC5883_REG_DATA);
	Wire.endTransmission();

	// demand 6 bytes from the register we set earlier
	Wire.requestFrom(HMC5883_ADDRESS, 6);

	// here we wait for the 6 bytes and then convert them
	// into something usable using bit shifting
	while(Wire.available() < 6);
        //Serial.println("hello");
	x = Wire.read()<<8; //X msb
	x |= Wire.read(); //X lsb
	z = Wire.read()<<8; //Z msb
	z |= Wire.read(); //Z lsb
	y = Wire.read()<<8; //Y msb
	y |= Wire.read(); //Y lsb
	
	// get a heading in radians
	heading = atan2(x, y);

	// return the heading in degrees, 57.29582 = 180 / PI
	return heading * 57.29582;
}




////nolonger needed code

//my old maths that doesn't work
/*
 if (angle > NO_ANGLE)
      {
        angle += 5;    
      } 
      else if (angle < NO_ANGLE)
      {
        angle -= 5;     
      }
  
      if (angle > MAX_ANGLE)
      {
        angle = MAX_ANGLE;    
      } 
      else if (angle < MIN_ANGLE)
      {
        angle = MIN_ANGLE;     
      }   */

//--------------------------------------------------//
//  The GPS code                                    //
//--------------------------------------------------//

/**
 * //TODO : setup and check it is avaliable ect. see http://arduiniana.org/libraries/tinygps/
 * @returns : array containing the latitude and longitude. 
**/
/**long* getPosition()
{
  double lat, lon;
  //unsigned long fixAge;
  //gps.get_position(&lat, &lon, &fixAge);
  
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  
  double currentPosition[2] = {lat, lon};
  return currentPosition;
}**/

