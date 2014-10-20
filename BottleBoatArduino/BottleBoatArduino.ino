#include <Wire.h>
#include <Servo.h>
#include "TinyGPS.h" // Downloaded from : https://github.com/mikalhart/TinyGPS/releases/tag/v13

Servo servo;
TinyGPS gps; // We are using a EM-408 GPS.  Also see http://arduiniana.org/libraries/tinygps/

//list of arduino pins the devices are conected to. 
int const motorPin = 9;
int const servoPin = 10;

long wayPoints[][2] = { {1,2}, {3,4} }; //{ {latitude, longitude}, .... } the points we want the boat to sail too.


/**
 *
**/
void setup()
{
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
 //TODO 
}


/**
 * @Description: Anything to do with controlling the servo.
 * @param angle int[in] the angle the rudder should be set to
**/
void setDirection(int angle)
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
//  The GPS code                                    //
//--------------------------------------------------//

/**
 *
 * @returns : array containing the latitude and longitude. //TODO : check it is avaliable ect. see http://arduiniana.org/libraries/tinygps/
**/
long* getPosition()
{
  long lat, lon;
  unsigned long fixAge;
  gps.get_position(&lat, &lon, &fixAge);
  
  long wayPoint[2] = {lat, lon};
  return wayPoint;
}




//--------------------------------------------------//
//  The Compass                                     //
//--------------------------------------------------//
#define HMC5883_ADDRESS				0x1E
#define HMC5883_REG_DATA			0x03
#define HMC5883_REG_WRITE_MODE			0x00

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
        Wire.write(HMC5883_REG_DATA); //select mode register
        Wire.write(HMC5883_REG_WRITE_MODE); //continuous measurement mode
        Wire.endTransmission();
	//Wire.write(HMC5883_ADDRESS, HMC5883_REG_WRITE_MODE, 0x00); 
	Serial.println("HMC5883 set to continous mode");
	delay(50);
}

/**
 * Returns the forward heading of the compass
 */
int get_heading()
{
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
