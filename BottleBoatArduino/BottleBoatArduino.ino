#define DEBUG

//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
//#include "TinyGPS.h" // Downloaded from : https://github.com/mikalhart/TinyGPS/releases/tag/v13
#include "softare_uart.c"

TinyGPSPlus gps; // We are using a EM-408 GPS.  Also see http://arduiniana.org/libraries/tinygps/
Servo servo;

//list of arduino pins the devices are conected to. 
int const motorPin = 6;
int const servoPin = 5;
int const gpsPin = 11;
int const gpsBaud = 4800;

int const MAX_ANGLE = 180;
int const MIN_ANGLE = 0;
int const NO_ANGLE = 90;


double waypoints[][2] = { {52.1,-4.1}, {51.5,-4.1} }; //{ {latitude, longitude}, .... } the points we want the boat to sail too.
int currentWaypointIndex = 0;

/**
 *
**/
void setup()
{
  Serial.begin(115200);

  //setup servo/rudder
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin, 620, 2400);//, 1060, 1920);
  digitalWrite(servoPin, LOW);  
  setDirection(NO_ANGLE);
 
  //setup GPS
  pinMode(gpsPin, INPUT);
 
  //setup motor 
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);
 
  //setup compass
  init_compass();
}


/**
 * @Description: The main part of the program
**/
void loop()
{
  // get the GPS position
  waitForGpsPosition();
  float currentLat = gps.location.lat();//52.07;
  float currentLng = gps.location.lng();//-4.02;    
  Serial.print("gps.location.lat() = ");
  Serial.println(currentLat);
  Serial.print("gps.location.lng() = ");
  Serial.println(currentLng);
  
  // get how far away we are  
  double waypointLat = waypoints[currentWaypointIndex][0];
  double waypointLng = waypoints[currentWaypointIndex][1];    
  double distanceKm = gps.distanceBetween(currentLat, currentLng, waypointLat, waypointLng) / 1000.0; //distance_between(currentLat, currentLng, waypointLat, waypointLng) / 1000.0;  //(gps.location.lat(), gps.location.lng(), waypointLat, waypointLng) / 1000.0;    
    
  #ifdef DEBUG
  Serial.print("distanceKm = ");
  Serial.println(distanceKm);
  #endif
    
  // check if we have already reached the waypoint
  if (distanceKm < 0.014)// we will try to get within 14meters of the target.
  {
    Serial.print("Reached : ");
    Serial.print(waypoints[currentWaypointIndex][0]);
    Serial.print(", ");
    Serial.println(waypoints[currentWaypointIndex][1]);
      
    currentWaypointIndex++;   
    // will error once we have finished ;)
    waypointLat = waypoints[currentWaypointIndex][0];
    waypointLng = waypoints[currentWaypointIndex][1];    
     
    Serial.print("Go to : ");
    Serial.print(waypoints[currentWaypointIndex][0]);
    Serial.print(", ");
    Serial.println(waypoints[currentWaypointIndex][1]); 
  }     
  
  //get the angle we WANT to head in and get the angle we are heading in  
  double courseTo = gps.courseTo(currentLat,currentLng, waypointLat, waypointLng);  //course_to(currentLat, currentLng, waypointLat, waypointLng);//(gps.location.lat(),gps.location.lng(), waypointLat, waypointLng);  
  int currentHeading = get_heading(); 
  #ifdef DEBUG
    Serial.print("courseTo = ");
    Serial.println(courseTo); 
    Serial.print("currentHeading = ");
    Serial.println(currentHeading); 
  #endif
    
  // set the angle of the servo, if we are heading in the wrong direction   
  double relativeHeading = (courseTo - currentHeading);//fakeAngle);
  double relativeHeadingClamped = relativeHeading;
  if(relativeHeadingClamped < -90) 
  {
    relativeHeadingClamped = -90;
  }
  else if(relativeHeadingClamped > 90) 
  {
    relativeHeadingClamped = 90;
  }
  double angle = NO_ANGLE + relativeHeadingClamped;
  setDirection(angle);
    
  #ifdef DEBUG
    Serial.print("relativeHeading = ");
    Serial.println(relativeHeading);
    Serial.print("relativeHeadingClamped = ");
    Serial.println(relativeHeadingClamped);
    Serial.print("Servo angle is : ");
    Serial.println(angle);
  #endif          
 
  // set the motor speed
  if ((angle > 10) || (distanceKm < 0.004)) //if tight turn, or close to target then move slower
  {
    setMotorSpeed(100); 
  }
  else
  {      
    setMotorSpeed(270);       
  }   
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

/**
 * @Description: Anything to do with controlling the motor
 *
**/
void setMotorSpeed(int motorSpeed)
{
  analogWrite(motorPin, motorSpeed);
}


//--------------------------------------------------//
//  The GPS                                     //
//--------------------------------------------------//
void waitForGpsPosition()
{
  char gps_string[255]; 
  int i=0;
  gps_string[0]=0;
  char gps_char;
  boolean started=false;
  while(i<254) 
  {
    gps_char = uart_rx(gpsPin,gpsBaud);
    if(gps_char=='$')
    {
      started=true;
    }
    if(started)
    {
      gps_string[i] = gps_char;
      i++;
      if(gps_char=='\r')
      {
        break;
      }
    }
  }
  gps_string[i] = 0;
  Serial.print("pre gps_string = ");
  Serial.println(gps_string);
  if(gps_string[1]=='G'&&gps_string[2]=='P'&&gps_string[3]=='R'&&gps_string[4]=='M'&&gps_string[5]=='C')
  {
    for(i=0;i<strlen(gps_string);i++)
    {
      gps.encode(gps_string[i]);
    }
    Serial.print("gps_string = ");
    Serial.println(gps_string);
  }   
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
  int x, y, z;
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

  x = Wire.read()<<8; //X msb
  x |= Wire.read(); //X lsb
  z = Wire.read()<<8; //Z msb
  z |= Wire.read(); //Z lsb
  y = Wire.read()<<8; //Y msb
  y |= Wire.read(); //Y lsb
	
  // get a heading in radians
  heading = atan2(x, y);

  // return the heading in degrees, 57.29582 = 180 / PI
  heading = heading * 57.29582;
  if (heading < 0)
  {
    heading+= 360;
  }
  return heading;
}


