#ifndef _MEGA128_USART0_NMEA_H_
#define _MEGA128_USART0_NMEA_H_

typedef unsigned char BOOL;
#define TRUE 0xFF
#define FALSE 0x00
#define MAX_NMEA_SENTENCE_LENGTH 80
#define EARTH_RADIUS_AVERAGE 6371.230
#define WGS84_A 6378.137
#define WGS84_B 6356.752314245
#define WGS84_F (WGS84_A-WGS84_B)/WGS84_A
#define UTC_KOREA_DIFFERENCE 9
#define DegreeOut(min) (min)/60.0
#define MinuteOut(deg) (deg)*60.0

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//char nmeaSentenceExample[MAX_NMEA_SENTENCE_LENGTH]="$GPRMC,032217.00,A,3736.20945,N,12700.61014,E,1.357,,240623,,,A*79";

typedef struct tagGPS {
	char nmeaSentence[MAX_NMEA_SENTENCE_LENGTH];
	char LatitudeArr[16];
	char LongitudeArr[16];
	char NMEADate[16];
	char NMEATime[16];
	double LatitudeValue;
	double LongitudeValue;
} GPS;

typedef struct tagUTC {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} UTC;

void USART0_Init(unsigned int ubrr);
unsigned char USART0_Receive(void);
void CH0_GetGPRMC(char *nmeaSentence);
void GPRMC_SplitCoordinate(char *nmeaSentence, char *Latitude, char *Longitude);
void GPRMC_SplitUTCTime(char *nmeaSentence, char *UTCTime);
void GPRMC_SplitUTCDate(char *nmeaSentence, char *UTCDate);
double LatitudeTransformToDouble(char *Latitude);
double LongitudeTransformToDouble(char *Longitude);
double GPS_HaversineDistanceOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2);
double GPS_HaversineAzimuthOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2);
double GPS_VincentyDistanceOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2);
double GPS_VincentyAzimuthOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2);
BOOL Check_leapyear(uint16_t year);
void Update_UTC_origin(char *UTCDate, char *UTCTime);
void Update_UTC_korea(UTC *origin);

UTC origin;
UTC korea;
BOOL flag_leapyear = FALSE;

uint8_t last_day[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

void USART0_Init(unsigned int ubrr) {
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

unsigned char USART0_Receive(void) {
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)))
	;
	/* Get and return received data from buffer */
	return UDR0;
}

void CH0_GetGPRMC(char *nmeaSentence) {
	char buffer[2*MAX_NMEA_SENTENCE_LENGTH];
	unsigned char data;
	unsigned char stack_RX;
	unsigned char index_GPRMC=0;
	do {
		for(stack_RX=0; stack_RX<2*MAX_NMEA_SENTENCE_LENGTH; stack_RX++) {
			data=USART0_Receive();
			buffer[stack_RX]=(char)data;
		}
		while(1) {
			if(buffer[index_GPRMC]=='$') {
				break;
			}
			index_GPRMC++;
		}
	} while (buffer[index_GPRMC+3]!='R');
	for(unsigned char index=0; index<MAX_NMEA_SENTENCE_LENGTH; index++) {
		nmeaSentence[index]=buffer[index_GPRMC+index];
	}
}

void GPRMC_SplitCoordinate(char *nmeaSentence, char *Latitude, char *Longitude) {
	unsigned char stack_comma=0;
	while(1) {
		if(*nmeaSentence==',') {
			stack_comma++;
			if(stack_comma==3) {
				break;
			}
		}
		nmeaSentence++;
	}
	nmeaSentence++;
	for(unsigned char index_Lat=0; index_Lat<12; index_Lat++) {
		Latitude[index_Lat]=*nmeaSentence;
		nmeaSentence++;
	}
	Latitude[12]='\0';
	nmeaSentence++;
	for(unsigned char index_Lon=0; index_Lon<13; index_Lon++) {
		Longitude[index_Lon]=*nmeaSentence;
		nmeaSentence++;
	}
	Longitude[13]='\0';
}

void GPRMC_SplitUTCTime(char *nmeaSentence, char *UTCTime) {
	while(1) {
		if(*nmeaSentence==',') {
			break;
		}
		nmeaSentence++;
	}
	nmeaSentence++;
	for(unsigned char index_Time=0; index_Time<9; index_Time++) {
		UTCTime[index_Time]=*nmeaSentence;
		nmeaSentence++;
	}
	UTCTime[9]='\0';
}

void GPRMC_SplitUTCDate(char *nmeaSentence, char *UTCDate) {
	unsigned char stack_comma=0;
	while(1) {
		if(*nmeaSentence==',') {
			stack_comma++;
			if(stack_comma==9) {
				break;
			}
		}
		nmeaSentence++;
	}
	nmeaSentence++;
	for(unsigned char index_Date=0; index_Date<6; index_Date++) {
		UTCDate[index_Date]=*nmeaSentence;
		nmeaSentence++;
	}
	UTCDate[6]='\0';
}

double LatitudeTransformToDouble(char *Latitude) {
	char buffer[10];
	double LatitudeValue;
	double DegreeValue;
	double MinuteValue;
	
	for(unsigned char index=0; index<10; index++) {
		if(index<2) {
			buffer[index]=Latitude[index];
			} else {
			buffer[index]='\0';
		}
	}
	DegreeValue=(double)atoi(buffer);
	
	for(unsigned char index=0; index<10; index++) {
		if(index<8) {
			buffer[index]=Latitude[index+2];
			} else {
			buffer[index]='\0';
		}
	}
	MinuteValue=atof(buffer);
	DegreeValue+=DegreeOut(MinuteValue);
	LatitudeValue=DegreeValue;
	if(Latitude[11]=='S') {
		LatitudeValue=(-1.0)*LatitudeValue;
	}
	
	return LatitudeValue;
}

double LongitudeTransformToDouble(char *Longitude) {
	char buffer[10];
	double LongitudeValue;
	double DegreeValue;
	double MinuteValue;
	
	for(unsigned char index=0; index<10; index++) {
		if(index<3) {
			buffer[index]=Longitude[index];
			} else {
			buffer[index]='\0';
		}
	}
	DegreeValue=(double)atoi(buffer);
	
	for(unsigned char index=0; index<10; index++) {
		if(index<8) {
			buffer[index]=Longitude[index+3];
			} else {
			buffer[index]='\0';
		}
	}
	MinuteValue=atof(buffer);
	DegreeValue+=DegreeOut(MinuteValue);
	LongitudeValue=DegreeValue;
	if(Longitude[12]=='W') {
		LongitudeValue=(-1.0)*LongitudeValue;
	}
	
	return LongitudeValue;
}

/*Haversine's formula*/
double GPS_HaversineDistanceOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2) {
	double Distance;
	double DifLat;
	double DifLon;
	LatitudeValue1=LatitudeValue1*M_PI/180.0;
	LatitudeValue2=LatitudeValue2*M_PI/180.0;
	LongitudeValue1=LongitudeValue1*M_PI/180.0;
	LongitudeValue2=LongitudeValue2*M_PI/180.0;
	DifLat=LatitudeValue2-LatitudeValue1;
	DifLon=LongitudeValue2-LongitudeValue1;

	Distance=2*EARTH_RADIUS_AVERAGE*asin(sqrt(pow(sin(DifLat/2.0),2)+cos(LatitudeValue1)*cos(LatitudeValue2)*pow(sin(DifLon/2.0),2)));
	
	return Distance;
}

double GPS_HaversineAzimuthOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2) {
	double Azimuth;
	double DifLon;
	double Xvalue;
	double Yvalue;
	LatitudeValue1=LatitudeValue1*M_PI/180.0;
	LatitudeValue2=LatitudeValue2*M_PI/180.0;
	LongitudeValue1=LongitudeValue1*M_PI/180.0;
	LongitudeValue2=LongitudeValue2*M_PI/180.0;
	DifLon=LongitudeValue2-LongitudeValue1;
	Yvalue=sin(DifLon)*cos(LatitudeValue2);
	Xvalue=cos(LatitudeValue1)*sin(LatitudeValue2)-sin(LatitudeValue1)*cos(LatitudeValue2)*cos(DifLon);
	Azimuth=atan2(Yvalue,Xvalue);
	Azimuth=(Azimuth*180.0/M_PI+360.0);
	if(Azimuth>=360.0) {
		Azimuth-=360.0;
	}
	
	return Azimuth;
}

/*Vincenty's formula*/
double GPS_VincentyDistanceOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2) {
	LatitudeValue1 = M_PI * LatitudeValue1 / 180.0;
	LatitudeValue2 = M_PI * LatitudeValue2 / 180.0;
	LongitudeValue1 = M_PI * LongitudeValue1 / 180.0;
	LongitudeValue2 = M_PI * LongitudeValue2 / 180.0;
	
	const double L = LongitudeValue2 - LongitudeValue1;
	const double tanU1 = (1-WGS84_F) * tan(LatitudeValue1);
	const double tanU2 = (1-WGS84_F) * tan(LatitudeValue2);
	const double cosU1 = 1 / sqrt(1 + tanU1*tanU1);
	const double cosU2 = 1 / sqrt(1 + tanU2*tanU2);
	const double sinU1 = tanU1 * cosU1;
	const double sinU2 = tanU2 * cosU2;
	
	double lamda = LongitudeValue2 - LongitudeValue1;
	double sinLamda, cosLamda;
	double sigma, sinSigma, cosSigma;
	double cos2sigma_m;
	double cosSqx;
	double lamdaPrime;
	
	do {
		sinLamda = sin(lamda);
		cosLamda = cos(lamda);
		const double sinSqSigma = (cosU2*sinLamda)*(cosU2*sinLamda)+(cosU1*sinU2-sinU1*cosU2*cosLamda)*(cosU1*sinU2-sinU1*cosU2*cosLamda);
		sinSigma = sqrt(sinSqSigma);
		cosSigma = sinU1*sinU2+cosU1*cosU2*cosLamda;
		sigma = atan2(sinSigma, cosSigma);
		const double sinAlpha = cosU1*cosU2*sinLamda/sinSigma;
		cosSqx = 1-sinAlpha*sinAlpha;
		cos2sigma_m = cosSigma-2*sinU1*sinU2/cosSqx;
		const double C = (WGS84_F/16)*cosSqx*(4+WGS84_F*(4-3*cosSqx));
		lamdaPrime = lamda;
		lamda = L+(1-C)*WGS84_F*sinAlpha*(sigma+C*sinSigma*(cos2sigma_m+C*cosSigma*(-1.0+2.0*cos2sigma_m*cos2sigma_m)));
	} while(abs(lamda-lamdaPrime) > pow(10.0, -13));
	
	const double uSq = cosSqx*(WGS84_A*WGS84_A-WGS84_B*WGS84_B)/(WGS84_B*WGS84_B);
	const double A = 1+(uSq/16384.0)*(256.0+uSq*(-128.0+uSq*(320.0-172.0*uSq)));
	const double B = uSq/1024.0 * (256.0+uSq*(-128.0+uSq*(74.0-47.0*uSq)));
	const double deltaSigma = B*sinSigma*(cos2sigma_m + (B/4.0)*(cosSigma*(-1+2*cos2sigma_m*cos2sigma_m)-(B/6.0)*cos2sigma_m*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2sigma_m*cos2sigma_m)));
	
	const double s = WGS84_B*A*(sigma-deltaSigma);
	
	return s;
}

double GPS_VincentyAzimuthOut(double LatitudeValue1, double LatitudeValue2, double LongitudeValue1, double LongitudeValue2) {
	LatitudeValue1 = M_PI * LatitudeValue1 / 180.0;
	LatitudeValue2 = M_PI * LatitudeValue2 / 180.0;
	LongitudeValue1 = M_PI * LongitudeValue1 / 180.0;
	LongitudeValue2 = M_PI * LongitudeValue2 / 180.0;
	
	const double L = LongitudeValue2 - LongitudeValue1;
	const double tanU1 = (1-WGS84_F) * tan(LatitudeValue1);
	const double tanU2 = (1-WGS84_F) * tan(LatitudeValue2);
	const double cosU1 = 1 / sqrt(1 + tanU1*tanU1);
	const double cosU2 = 1 / sqrt(1 + tanU2*tanU2);
	const double sinU1 = tanU1 * cosU1;
	const double sinU2 = tanU2 * cosU2;
	
	double lamda = LongitudeValue2 - LongitudeValue1;
	double sinLamda, cosLamda;
	double sigma, sinSigma, cosSigma;
	double cos2sigma_m;
	double cosSqx;
	double lamdaPrime;
	
	do {
		sinLamda = sin(lamda);
		cosLamda = cos(lamda);
		const double sinSqSigma = (cosU2*sinLamda)*(cosU2*sinLamda)+(cosU1*sinU2-sinU1*cosU2*cosLamda)*(cosU1*sinU2-sinU1*cosU2*cosLamda);
		sinSigma = sqrt(sinSqSigma);
		cosSigma = sinU1*sinU2+cosU1*cosU2*cosLamda;
		sigma = atan2(sinSigma, cosSigma);
		const double sinAlpha = cosU1*cosU2*sinLamda/sinSigma;
		cosSqx = 1-sinAlpha*sinAlpha;
		cos2sigma_m = cosSigma-2*sinU1*sinU2/cosSqx;
		const double C = (WGS84_F/16)*cosSqx*(4+WGS84_F*(4-3*cosSqx));
		lamdaPrime = lamda;
		lamda = L+(1-C)*WGS84_F*sinAlpha*(sigma+C*sinSigma*(cos2sigma_m+C*cosSigma*(-1.0+2.0*cos2sigma_m*cos2sigma_m)));
	} while(abs(lamda-lamdaPrime) > pow(10.0, -13));
	
	const double alpha1 = atan2(cosU2*sinLamda, cosU1*sinU2-sinU1*cosU2*cosLamda)*(180.0/M_PI);
	
	return alpha1;
}

BOOL Check_leapyear(uint16_t year) {
	if (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0)) {
		return TRUE;
		} else {
		return FALSE;
	}
}

void Update_UTC_origin(char *UTCDate, char *UTCTime) {
	char buffer[2];
	
	buffer[0]=UTCDate[4];
	buffer[1]=UTCDate[5];
	origin.year=atoi(buffer);
	
	buffer[0]=UTCDate[2];
	buffer[1]=UTCDate[3];
	origin.month=atoi(buffer);
	
	buffer[0]=UTCDate[0];
	buffer[1]=UTCDate[1];
	origin.day=atoi(buffer);
	
	buffer[0]=UTCTime[0];
	buffer[1]=UTCTime[1];
	origin.hour=atoi(buffer);
	
	buffer[0]=UTCTime[2];
	buffer[1]=UTCTime[3];
	origin.minute=atoi(buffer);
	
	buffer[0]=UTCTime[4];
	buffer[1]=UTCTime[5];
	origin.second=atoi(buffer);
}

void Update_UTC_korea(UTC *origin) {
	korea.second = origin->second;
	korea.minute = origin->minute;
	korea.hour = (origin->hour) + UTC_KOREA_DIFFERENCE;
	korea.day = origin->day;
	korea.month = origin->month;
	korea.year = origin->year;
	
	if(korea.hour >= 24) {
		korea.hour -= 24;
		korea.day += 1;
		if(korea.day > last_day[(origin->month)-1]) {
			korea.day = 1;
			korea.month += 1;
			if(korea.month > 12) {
				korea.month = 1;
				korea.year += 1;
			}
		}
	}
}

#endif