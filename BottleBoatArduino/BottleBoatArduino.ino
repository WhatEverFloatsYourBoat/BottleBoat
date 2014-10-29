//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
//#include "TinyGPS.h" // Downloaded from : https://github.com/mikalhart/TinyGPS/releases/tag/v13
#include "softare_uart.c"

TinyGPSPlus gps; // We are using a EM-408 GPS.  Also see http://arduiniana.org/libraries/tinygps/
//TinyGPS gps;
//SoftwareSerial ss(11,12); // For the GPS. 4,3 are the pins that the GPS are attached too.
Servo servo;
//list of arduino pins the devices are conected to. 
int const motorPin = 6;
int const servoPin = 5;
int const gpsPin = 11;
int const gpsBaud = 4800;

//int const MAX_ANGLE = 270;
//int const MIN_ANGLE = 100;
//int const NO_ANGLE = 180;
int const MAX_ANGLE = 180;
int const MIN_ANGLE = 0;
int const NO_ANGLE = 90;

//int fakeAngle = 0;

double waypoints[][2] = { {52.0,-4.1}, {51.5,-4.1} }; //{ {latitude, longitude}, .... } the points we want the boat to sail too.
int currentWaypointIndex = 0;

/**
 *
**/
void setup()
{
  Serial.begin(115200);
  //ss.begin(4800);
 //setup servo/rudder
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin, 620, 2400);//, 1060, 1920);
  digitalWrite(servoPin, LOW);  
  setDirection(NO_ANGLE);
 
 //servo.writeMicroseconds(1500);
 //servo.write(50);
 setDirection(NO_ANGLE);
 
 
 pinMode(gpsPin, INPUT);
 //setup motor 
 //pinMode(motorPin, OUTPUT);
 //digitalWrite(motorPin, LOW);
 
 //setup compass
 init_compass();
}


/**
 * @Description: The main part of the program
**/
void loop()
{
  //Serial.println(get_heading());
  //return;
  
  
  //fakeAngle = (fakeAngle + 1) % 360;
  //return;
  //Serial.println("loop"); 
 /* while (true){
    if(ss.available() > 0)
    {
      //Serial.print();
    Serial.write(ss.read());
    }
  }*/
  
  //char gps_char;
  //gps_char = uart_rx(gpsPin, gpsBaud);
  //boolean gps_ready = false;
  char gps_string[255];
  int i=0;
  gps_string[0]=0;
  char gps_char;
  boolean started=false;
  while(i<254) {
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
  
 /* unsigned long start = millis();
  while(millis() < start +2000)
  {
   if (char c = uart_rx(11,4800))
  {
   gps.encode(c);
   Serial.print(c);
   if(c == '/n')
   {
    break; 
   }
  } 
    
  }*/
  
  float currentLat = 52.07;//gps.location.lat();
  float currentLng = -4.02;//gps.location.lng();
  
  Serial.print("gps.location.lat() = ");
    Serial.println(currentLat);//currentLat);//location.lat());
    Serial.print("gps.location.lng() = ");
    Serial.println(currentLng);//currentLng);//gps.location.lng());
   // return;*/
 
  //Serial.println();

  //while (!gps.encode(uart_rx(gpsPin, gpsBaud)));
  
  //gps.encode(ss.read()); 
  
 // if(ss.available() > 0)
 // {
    //if (gps.location.isUpdated())//gps.encode(ss.read()))// 
    //if(gps.location.isUpdated())
    //{
    //Serial.println("ss avaliable");
    //setDirection(angle);
    
     // at the start of each loop we want to get what the GPS coordinates 
    
    double waypointLat = waypoints[currentWaypointIndex][0];
    double waypointLng = waypoints[currentWaypointIndex][1];
    //float currentLat, currentLng;
    unsigned long fix_age;
    //gps.f_get_position(&currentLat, &currentLng, &fix_age);
    // check if we have already reached the waypoint
    Serial.print("gps.location.lat() = ");
    Serial.println(currentLat);//currentLat);//location.lat());
    Serial.print("gps.location.lng() = ");
    Serial.println(currentLng);//currentLng);//gps.location.lng());
    
    double distanceKm = gps.distanceBetween(currentLat, currentLng, waypointLat, waypointLng) / 1000.0; //distance_between(currentLat, currentLng, waypointLat, waypointLng) / 1000.0;  //(gps.location.lat(), gps.location.lng(), waypointLat, waypointLng) / 1000.0;    
    Serial.print("distanceKm = ");
    Serial.println(distanceKm);
    
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
     
    
    double courseTo = gps.courseTo(currentLat,currentLng, waypointLat, waypointLng);  //course_to(currentLat, currentLng, waypointLat, waypointLng);//(gps.location.lat(),gps.location.lng(), waypointLat, waypointLng);  
    Serial.print("courseTo = ");
    Serial.println(courseTo);  
    int currentHeading = get_heading();
    Serial.print("currentHeading = ");
    Serial.println(currentHeading); 
    // set the angle of the servo, if we are heading in the wrong direction
   // if ( checkHeading(courseTo, currentHeading) )
    //{         
      //int angleDiff = courseTo - currentHeading;
      //int angleMin = (angleDiff + 180) % 360 - 180;
      //Serial.print("angleDiff is : ");
      //Serial.println(angleDiff);
      //Serial.print("angleMin : ");
      //Serial.println(angleMin);
      //Serial.println((angleDiff + 180) % 360 - 180);
      //int angle = (sq((angleDiff + 180) % 360 - 180))/1620; // sq(180)/80 = 405 : We will want to turn a max of 180degrees, and we never want servo/rudder to go above 80 (don't wnat your rudder back-to-front or flat against boat). 
      //angle += 5; // just to make sure you are actually going to be turning. 
      //int angle = angleMin + 5;
      //int angle = angleMin;
      //Serial.print("pre-angle is : ");
      //Serial.println(angle);
      //if ((angleDiff < 0) && (angleDiff > -180))//turn anticlockwise
      //{        
        //Serial.print("turn anticlockwise");
        //angle = NO_ANGLE + angle;
      //}
      //else //angleDiff > 0 and angleDiff < 180 //turn clockwise
      //{
        //Serial.print("turn clockwise");
        //angle = NO_ANGLE - angle; 
        
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
              
      //setMotorSpeed(100); //slowish speed wanted while turning  
   // }   
    if ((angle > 10) || (distanceKm < 0.004)) 
    {
      setMotorSpeed(100); 
    }
    else
    {
      
        setMotorSpeed(270); 
      
    }   
    
  //}
}

/**
*
**/
/*boolean checkHeading(double courseTo, int currentHeading)
{
  int angle = courseTo - currentHeading;
  //angle = (angle + 180) % 360 - 180;
 
  //return angle;
  
  return true;//((abs((angle + 180) % 360 - 180)) > 5);  
}*/



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
        heading = heading * 57.29582;
        if (heading < 0)
        {
          heading+= 360;
        }
	return heading;
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

