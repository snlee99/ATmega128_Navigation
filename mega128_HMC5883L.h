#ifndef _MEGA128_HMC5883L_H_
#define _MEGA128_HMC5883L_H_

#define HMC5883L_ADDR 0x1E
#define HMC5883L_RegisterA 0x00
#define HMC5883L_RegisterB 0x01
#define HMC5883L_ModeRegister 0x02
#define HMC5883L_XMSB 0x03
#define HMC5883L_XLSB 0x04
#define HMC5883L_ZMSB 0x05
#define HMC5883L_ZLSB 0x06
#define HMC5883L_YMSB 0x07
#define HMC5883L_YLSB 0x08

//-09d05m(2024.06.27 South Korea, Seoul)
#define MDA_degree_korea -9
#define MDA_minute_korea 5

// Set declination angle on your location and fix heading
// You can find your declination on: http://magnetic-declination.com/
// (+) Positive or (-) for negative
// For Bytom / Poland declination angle is 4'26E (positive)
// Formula: (deg + (min / 60.0)) / (180 / M_PI);

#include <math.h>
#include "mega128_I2C.h"

typedef struct tagMF {
	int16_t Xvalue;
	int16_t Yvalue;
	int16_t Zvalue;
	volatile double Heading;
} MF;

void HMC5883L_ContinuousInit(void);
double HMC5883L_AzimuthRead(void);
double DeclinationAngleOut(short degree, short minute);

//-08d59m(2023.04.23 South Korea, Seoul)
short MDA_degree = -8, MDA_minute = 59;
double MagneticDeclinationAngle = 0;

void HMC5883L_ContinuousInit(void) {
	i2c_start();
	i2c_write(HMC5883L_ADDR<<1|I2C_WRITE);
	i2c_write(HMC5883L_RegisterA);
	i2c_write(0x70); //8 average, 15Hz
	i2c_stop();
	
	i2c_start();
	i2c_write(HMC5883L_ADDR<<1|I2C_WRITE);
	i2c_write(HMC5883L_RegisterB);
	i2c_write(0xA0); //Digital Resolution 2.56mG/LSb
	i2c_stop();
	
	i2c_start();
	i2c_write(HMC5883L_ADDR<<1|I2C_WRITE);
	i2c_write(HMC5883L_ModeRegister);
	i2c_write(0x00); //Continuous-Measurement Mode
	i2c_stop();
}

double HMC5883L_AzimuthRead(void) {
	int16_t magnetic_field_X, magnetic_field_Y, magnetic_field_Z;
	double Azimuth;
	i2c_start();
	i2c_write(HMC5883L_ADDR<<1|I2C_WRITE);
	i2c_write(HMC5883L_XMSB);
	i2c_stop();
	
	i2c_start();
	i2c_write(HMC5883L_ADDR<<1|I2C_READ);
	magnetic_field_X=(i2c_read(ACK)<<8);
	magnetic_field_X|=i2c_read(ACK);
	magnetic_field_Z=(i2c_read(ACK)<<8);
	magnetic_field_Z|=i2c_read(ACK);
	magnetic_field_Y=(i2c_read(ACK)<<8);
	magnetic_field_Y|=i2c_read(NACK);
	i2c_stop();
	
	Azimuth=atan2((double)magnetic_field_Y, (double)magnetic_field_X);
	Azimuth+=MagneticDeclinationAngle;
	if(Azimuth<0) {
		Azimuth+=2.0*M_PI;
	}
	if(Azimuth>2*M_PI) {
		Azimuth-=2.0*M_PI;
	}
	Azimuth=Azimuth*180.0/M_PI;
	
	return Azimuth;
}

double DeclinationAngleOut(short degree, short minute) {
	return ((double)degree + ((double)minute / 60.0)) / (180.0 / M_PI);
}

#endif