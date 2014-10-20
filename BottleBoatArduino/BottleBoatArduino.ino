#include <Wire.h>
#include "TinyGPS.h"
TinyGPS gps; // See http://arduiniana.org/libraries/tinygps/

void setup()
{
 //TODO 
}

void loop()
{
 //TODO 
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
	Wire.write(HMC5883_ADDRESS, HMC5883_REG_WRITE_MODE, 0x00); //TODO : Bug in this line
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
