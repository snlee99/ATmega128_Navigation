#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR (F_CPU/(16UL*BAUD)-1)
#ifndef sbi
#define sbi(reg, bit) (reg) |= (1<<(bit))
#endif
#ifndef cbi
#define cbi(reg, bit) (reg) &= ~(1<<(bit))
#endif
#define LED_DDR DDRG
#define LED_PORT PORTG
#define NEGATIVE_LED_CH 0
#define COMMAND_LED_CH 1

#include <avr/io.h>
#include <util/delay.h>
#include "mega128_4x3Keymatrix.h"
#include "mega128_I2C.h"
#include "mega128_HMC5883L.h"
#include "mega128_PCF8574.h"
#include "mega128_USART0_NMEA.h"
#include "mega128_SSD1309.h"

typedef struct tagMapData {
	volatile double Distance;
	volatile double Bearing;
} MapData;

void NegativeLED_Init(void);
void CommandLED_Init(void);
double LatitudeExtract(char *buffer_latitude, BOOL flag_negative);
double LongitudeExtract(char *buffer_longitude, BOOL flag_negative);
void DeclinationAngleUpdate(char *buffer_MDA, BOOL flag_negative);

int main(void) {
	//System initializing code start
	char key='X';
	char buffer[20];
	BOOL flag_negative=FALSE;
	BOOL flag_command=FALSE;
	BOOL flag_vincenty=FALSE;
	BOOL flag_korea=FALSE;
	GPS MyGPS;
	GPS TargetGPS;
	GPS CursorGPS;
	MF MyMF;
	MapData MyMapData;
	Pixel MyPixel;
	Pixel TargetPixel;
	Pixel CursorPixel;
	short previousMDA_degree = 0, previousMDA_minute = 0;
	MagneticDeclinationAngle = DeclinationAngleOut(MDA_degree, MDA_minute);
	
	NegativeLED_Init();
	CommandLED_Init();
	keymatrix_init();
	i2c_init();
	Initialize_LCD();
	HMC5883L_ContinuousInit();
	SSD1309_Init();
	USART0_Init(MYUBRR);
	//System initializing code stop
	
	while (1) {
		//Initial state screen updating code start
		key=keymatrix_value();
		CH0_GetGPRMC(MyGPS.nmeaSentence);
		//sprintf(MyGPS.nmeaSentence, "$GPRMC,032217.00,A,3736.20945,N,12700.61014,E,1.357,,240623,,,A*79");
		GPRMC_SplitCoordinate(MyGPS.nmeaSentence, MyGPS.LatitudeArr, MyGPS.LongitudeArr);
		MyGPS.LatitudeValue = LatitudeTransformToDouble(MyGPS.LatitudeArr);
		MyGPS.LongitudeValue = LongitudeTransformToDouble(MyGPS.LongitudeArr);
		GPRMC_SplitUTCDate(MyGPS.nmeaSentence, MyGPS.NMEADate);
		GPRMC_SplitUTCTime(MyGPS.nmeaSentence, MyGPS.NMEATime);
		Update_UTC_origin(MyGPS.NMEADate, MyGPS.NMEATime);
		MyMF.Heading=HMC5883L_AzimuthRead();
		if(flag_korea == TRUE) {
			memmove(frameBuffer, koreaPeninsula, sizeof(koreaPeninsula));
			if((MyGPS.LatitudeValue>KOREA_PENINSULA_LAT_MIN) && (MyGPS.LatitudeValue<KOREA_PENINSULA_LAT_MAX) && (MyGPS.LongitudeValue>KOREA_PENINSULA_LON_MIN) && (MyGPS.LongitudeValue<KOREA_PENINSULA_LON_MAX)) {
				MyPixel.x=(uint8_t)((double)SSD1309_WIDTH*(MyGPS.LatitudeValue-KOREA_PENINSULA_LAT_MIN)/(KOREA_PENINSULA_LAT_MAX-KOREA_PENINSULA_LAT_MIN));
				MyPixel.y=(uint8_t)((double)SSD1309_HEIGHT*(MyGPS.LongitudeValue-KOREA_PENINSULA_LON_MIN)/(KOREA_PENINSULA_LON_MAX-KOREA_PENINSULA_LON_MIN));
				SSD1309_SetHorizon(MyPixel.y);
				SSD1309_SetVertical(MyPixel.x);
			}
			if(origin.month == 2) {
				flag_leapyear = Check_leapyear(origin.year);
				if(flag_leapyear == TRUE) {
					last_day[1] = 29;
					} else {
					last_day[1] = 28;
				}
			}
			Update_UTC_korea(&origin);
			sprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d", korea.year, korea.month, korea.day, korea.hour, korea.minute, korea.second);
		} else {
			memmove(frameBuffer, worldMap, sizeof(worldMap));
			MyPixel.x=(uint8_t)(((MyGPS.LongitudeValue/180.0)*((double)SSD1309_WIDTH/2.0)))+(SSD1309_WIDTH/2);
			MyPixel.y=(uint8_t)((MyGPS.LatitudeValue/90.0)*((double)SSD1309_HEIGHT/2.0))+(SSD1309_HEIGHT/2);
			MyPixel.y=SSD1309_HEIGHT-MyPixel.y;
			SSD1309_SetHorizon(MyPixel.y);
			SSD1309_SetVertical(MyPixel.x);
			sprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d", origin.year, origin.month, origin.day, origin.hour, origin.minute, origin.second);
		}
		SSD1309_UpdateScreen();
		LCD_position(0,0);
		LCD_string(buffer);
		sprintf(buffer, "MyLat: %08.5lf", MyGPS.LatitudeValue);
		LCD_position(1,0);
		LCD_string(buffer);
		sprintf(buffer, "MyLon: %09.5lf", MyGPS.LongitudeValue);
		LCD_position(2,0);
		LCD_string(buffer);
		sprintf(buffer, "Head: %06.2lf", MyMF.Heading);
		LCD_position(3,0);
		LCD_string(buffer);
		//Initial state screen updating code stop
		
		//Initial state CMD operating code start
		if(flag_command == TRUE && key == '1') { //CMD1: MDA change
			flag_command = FALSE;
			cbi(LED_PORT, COMMAND_LED_CH);
			LCD_clearFast();
			sprintf(buffer, "Current MDA: %02dd%02dm", MDA_degree, MDA_minute);
			LCD_position(0,0);
			LCD_string(buffer);
			
			char buffer_MDA[4]="XXXX";
			unsigned char index_MDA=0;
			do {
				key='X'; //Input numbers
				index_MDA=0;
				flag_negative=FALSE;
				cbi(LED_PORT, NEGATIVE_LED_CH);
				LCD_position(1,0);
				LCD_string("New MDA 4digits");
				LCD_rowClear(2);
				while(key != '*') {
					key=keymatrix_value();
					if((key != 'X') && (key != '*') && (key != '#')) {
						buffer_MDA[index_MDA]=key;
						LCD_position(2, index_MDA);
						I2C_LCD_data(buffer_MDA[index_MDA]);
						index_MDA++;
						_delay_ms(200); //New MDA key input delay
					} else if(key == '#') {
						flag_negative=~flag_negative;
						if(flag_negative==TRUE) {
							sbi(LED_PORT, NEGATIVE_LED_CH);
						} else {
							cbi(LED_PORT, NEGATIVE_LED_CH);
						}
						_delay_ms(200); //New MDA key input delay
					}
				}
			} while (buffer_MDA[3] == 'X');
			key='X'; //Right after pressing '*'
			DeclinationAngleUpdate(buffer_MDA, flag_negative);
			LCD_rowClear(2);
			sprintf(buffer, "%02dd%02dm", MDA_degree, MDA_minute);
			LCD_position(2,0);
			LCD_string(buffer);
			cbi(LED_PORT, NEGATIVE_LED_CH);
			_delay_ms(5000); //To the next stage
			LCD_clearFast();
		} else if(flag_command == TRUE && key == '2') { //CMD2: World Map/Korea Peninsula 
			key='X';
			flag_command = FALSE;
			cbi(LED_PORT, COMMAND_LED_CH);
			flag_korea = ~flag_korea;
			LCD_clearFast();
			if(flag_korea == TRUE) {
				previousMDA_degree = MDA_degree;
				previousMDA_minute = MDA_minute;
				MDA_degree = MDA_degree_korea;
				MDA_minute = MDA_minute_korea;
				LCD_position(0,0);
				LCD_string("Korea Peninsula ON");
				sprintf(buffer, "Time & Date: UTC+9");
				LCD_position(1,0);
				LCD_string(buffer);
			} else {
				MDA_degree = previousMDA_degree;
				MDA_minute = previousMDA_minute;
				LCD_position(0,0);
				LCD_string("World Map ON");
				sprintf(buffer, "Time & Date: UTC");
				LCD_position(1,0);
				LCD_string(buffer);
			}
			sprintf(buffer, "Updated MDA: %02dd%02dm", MDA_degree, MDA_minute);
			LCD_position(2,0);
			LCD_string(buffer);
			MagneticDeclinationAngle = DeclinationAngleOut(MDA_degree, MDA_minute);
			_delay_ms(5000); //To the next stage
			LCD_clearFast();
		}
		//Initial state CMD operating code stop
		
		if(key=='*') {
			//Target GPS Latitude input code start
			LCD_clearFast();
			cbi(LED_PORT, COMMAND_LED_CH);
			
			char buffer_latitude[7]="XXXXXXX";
			unsigned char index_latitude=0;
			do {
				key='X'; //Right after pressing '*' and Input numbers
				index_latitude=0;
				flag_negative=FALSE;
				cbi(LED_PORT, NEGATIVE_LED_CH);
				LCD_position(0,0);
				LCD_string("Latitude 7digits");
				LCD_rowClear(1);
				while(key != '*') {
					key=keymatrix_value();
					if((key != 'X') && (key != '*') && (key != '#')) {
						buffer_latitude[index_latitude]=key;
						LCD_position(1, index_latitude);
						I2C_LCD_data(buffer_latitude[index_latitude]);
						index_latitude++;
						_delay_ms(200); //Target GPS Latitude key input delay
					} else if(key == '#') {
						flag_negative=~flag_negative;
						if(flag_negative==TRUE) {
							sbi(LED_PORT, NEGATIVE_LED_CH);
						} else {
							cbi(LED_PORT, NEGATIVE_LED_CH);
						}
						_delay_ms(200); //Target GPS Latitude key input delay
					}
				}
			} while (buffer_latitude[6] == 'X');
			TargetGPS.LatitudeValue=LatitudeExtract(buffer_latitude, flag_negative);
			LCD_rowClear(1);
			sprintf(buffer, "%08.5lf", TargetGPS.LatitudeValue);
			LCD_position(1,0);
			LCD_string(buffer);
			//Target GPS Latitude input code stop
			
			//Target GPS Longitude input code start
			char buffer_longitude[8]="XXXXXXXX";
			unsigned char index_longitude=0;
			do {
				key='X'; //Right after pressing '*' and Input numbers
				index_longitude=0;
				flag_negative=FALSE;
				cbi(LED_PORT, NEGATIVE_LED_CH);
				LCD_position(2,0);
				LCD_string("Longitude 8digits");
				LCD_rowClear(3);
				while(key != '*') {
					key=keymatrix_value();
					if((key != 'X') && (key != '*') && (key != '#')) {
						buffer_longitude[index_longitude]=key;
						LCD_position(3,index_longitude);
						I2C_LCD_data(buffer_longitude[index_longitude]);
						index_longitude++;
						_delay_ms(200); //Target GPS Longitude key input delay
					} else if(key == '#') {
						flag_negative=~flag_negative;
						if(flag_negative==TRUE) {
							sbi(LED_PORT, NEGATIVE_LED_CH);
						} else {
							cbi(LED_PORT, NEGATIVE_LED_CH);
						}
						_delay_ms(200); //Target GPS Longitude key input delay
					}
				}
			} while (buffer_longitude[7] == 'X');
			TargetGPS.LongitudeValue=LongitudeExtract(buffer_longitude, flag_negative);
			LCD_rowClear(3);
			sprintf(buffer, "%09.5lf", TargetGPS.LongitudeValue);
			LCD_position(3,0);
			LCD_string(buffer);
			//Target GPS Longitude input code stop
			
			//After completing coordinates input, {key, CLCD, negative flag} reset code start
			key='X'; //Right after pressing '*'
			flag_negative = FALSE;
			cbi(LED_PORT, NEGATIVE_LED_CH);
			_delay_ms(5000); //To the next stage
			LCD_clearFast();
			//After completing coordinates input, {key, CLCD, negative flag} reset code stop
			
			while(key!='*') {
				//Tracking mode screen updating code start
				key=keymatrix_value();
				CH0_GetGPRMC(MyGPS.nmeaSentence);
				//sprintf(MyGPS.nmeaSentence, "$GPRMC,032217.00,A,3736.20945,N,12700.61014,E,1.357,,240623,,,A*79");
				GPRMC_SplitCoordinate(MyGPS.nmeaSentence, MyGPS.LatitudeArr, MyGPS.LongitudeArr);
				MyGPS.LatitudeValue = LatitudeTransformToDouble(MyGPS.LatitudeArr);
				MyGPS.LongitudeValue = LongitudeTransformToDouble(MyGPS.LongitudeArr);
				if(flag_vincenty == TRUE) {
					MyMapData.Distance=GPS_VincentyDistanceOut(MyGPS.LatitudeValue, TargetGPS.LatitudeValue, MyGPS.LongitudeValue, TargetGPS.LongitudeValue);
					MyMapData.Bearing=GPS_VincentyAzimuthOut(MyGPS.LatitudeValue, TargetGPS.LatitudeValue, MyGPS.LongitudeValue, TargetGPS.LongitudeValue);
					LCD_position(0,0);
					LCD_string("Vincenty mode ");
				} else {
					MyMapData.Distance=GPS_HaversineDistanceOut(MyGPS.LatitudeValue, TargetGPS.LatitudeValue, MyGPS.LongitudeValue, TargetGPS.LongitudeValue);
					MyMapData.Bearing=GPS_HaversineAzimuthOut(MyGPS.LatitudeValue, TargetGPS.LatitudeValue, MyGPS.LongitudeValue, TargetGPS.LongitudeValue);
					LCD_position(0,0);
					LCD_string("Haversine mode");
				}
				if(MyMapData.Distance < 1.0) {
					sprintf(buffer, "Dis: %011.5lfm",MyMapData.Distance*1000.0);
				} else {
					sprintf(buffer, "Dis: %011.5lfkm",MyMapData.Distance);
				}
				LCD_position(1,0);
				LCD_string(buffer);
				sprintf(buffer, "Bear: %06.2lf", MyMapData.Bearing);
				LCD_position(2,0);
				LCD_string(buffer);
				MyMF.Heading=HMC5883L_AzimuthRead();
				sprintf(buffer, "Head: %06.2lf", MyMF.Heading);
				LCD_position(3,0);
				LCD_string(buffer);
				
				if(flag_korea == TRUE) {
					memmove(frameBuffer, koreaPeninsula, sizeof(koreaPeninsula));
					if((MyGPS.LatitudeValue>KOREA_PENINSULA_LAT_MIN) && (MyGPS.LatitudeValue<KOREA_PENINSULA_LAT_MAX) && (MyGPS.LongitudeValue>KOREA_PENINSULA_LON_MIN) && (MyGPS.LongitudeValue<KOREA_PENINSULA_LON_MAX)) {
						MyPixel.x=(uint8_t)((double)SSD1309_WIDTH*(MyGPS.LatitudeValue-KOREA_PENINSULA_LAT_MIN)/(KOREA_PENINSULA_LAT_MAX-KOREA_PENINSULA_LAT_MIN));
						MyPixel.y=(uint8_t)((double)SSD1309_HEIGHT*(MyGPS.LongitudeValue-KOREA_PENINSULA_LON_MIN)/(KOREA_PENINSULA_LON_MAX-KOREA_PENINSULA_LON_MIN));
						SSD1309_SetHorizon(MyPixel.y);
						SSD1309_SetVertical(MyPixel.x);
					}
					if((TargetGPS.LatitudeValue>KOREA_PENINSULA_LAT_MIN) && (TargetGPS.LatitudeValue<KOREA_PENINSULA_LAT_MAX) && (TargetGPS.LongitudeValue>KOREA_PENINSULA_LON_MIN) && (TargetGPS.LongitudeValue<KOREA_PENINSULA_LON_MAX)) {
						TargetPixel.x=(uint8_t)((double)SSD1309_WIDTH*(TargetGPS.LatitudeValue-KOREA_PENINSULA_LAT_MIN)/(KOREA_PENINSULA_LAT_MAX-KOREA_PENINSULA_LAT_MIN));
						TargetPixel.y=(uint8_t)((double)SSD1309_HEIGHT*(TargetGPS.LongitudeValue-KOREA_PENINSULA_LON_MIN)/(KOREA_PENINSULA_LON_MAX-KOREA_PENINSULA_LON_MIN));
						SSD1309_SetHorizon(TargetPixel.y);
						SSD1309_SetVertical(TargetPixel.x);
					}
				} else {
					memmove(frameBuffer, worldMap, sizeof(worldMap));
					MyPixel.x=(uint8_t)(((MyGPS.LongitudeValue/180.0)*((double)SSD1309_WIDTH/2.0)))+(SSD1309_WIDTH/2);
					MyPixel.y=(uint8_t)((MyGPS.LatitudeValue/90.0)*((double)SSD1309_HEIGHT/2.0))+(SSD1309_HEIGHT/2);
					MyPixel.y=SSD1309_HEIGHT-MyPixel.y;
					SSD1309_SetHorizon(MyPixel.y);
					SSD1309_SetVertical(MyPixel.x);
					TargetPixel.x=(uint8_t)(((TargetGPS.LongitudeValue/180.0)*((double)SSD1309_WIDTH/2.0)))+(SSD1309_WIDTH/2);
					TargetPixel.y=(uint8_t)((TargetGPS.LatitudeValue/90.0)*((double)SSD1309_HEIGHT/2.0))+(SSD1309_HEIGHT/2);
					TargetPixel.y=SSD1309_HEIGHT-TargetPixel.y;
					SSD1309_SetHorizon(TargetPixel.y);
					SSD1309_SetVertical(TargetPixel.x);
				}
				SSD1309_UpdateScreen();
				//Tracking mode screen updating code stop
				
				//Tracking mode CMD operating code start
				if(flag_command == TRUE && key == '1') { //CMD1: MDA change
					flag_command = FALSE;
					cbi(LED_PORT, COMMAND_LED_CH);
					LCD_clearFast();
					sprintf(buffer, "Current MDA: %02dd%02dm", MDA_degree, MDA_minute);
					LCD_position(0,0);
					LCD_string(buffer);
				
					char buffer_MDA[4]="XXXX";
					unsigned char index_MDA=0;
					do {
						key='X'; //Input numbers
						index_MDA=0;
						flag_negative=FALSE;
						cbi(LED_PORT, NEGATIVE_LED_CH);
						LCD_position(1,0);
						LCD_string("New MDA 4digits");
						LCD_rowClear(2);
						while(key != '*') {
							key=keymatrix_value();
							if((key != 'X') && (key != '*') && (key != '#')) {
								buffer_MDA[index_MDA]=key;
								LCD_position(2, index_MDA);
								I2C_LCD_data(buffer_MDA[index_MDA]);
								index_MDA++;
								_delay_ms(200); //New MDA key input delay
							} else if(key == '#') {
								flag_negative=~flag_negative;
								if(flag_negative==TRUE) {
									sbi(LED_PORT, NEGATIVE_LED_CH);
									} else {
									cbi(LED_PORT, NEGATIVE_LED_CH);
								}
								_delay_ms(200); //New MDA key input delay
							}
						}
					} while (buffer_MDA[3] == 'X');
					key='X'; //Right after pressing '*'
					DeclinationAngleUpdate(buffer_MDA, flag_negative);
					LCD_rowClear(2);
					sprintf(buffer, "%02dd%02dm", MDA_degree, MDA_minute);
					LCD_position(2,0);
					LCD_string(buffer);
					cbi(LED_PORT, NEGATIVE_LED_CH);
					_delay_ms(5000); //To the next stage
					LCD_clearFast();
				} else if(flag_command == TRUE && key == '2') { //CMD2: World Map/Korea Peninsula
					key='X';
					flag_command = FALSE;
					cbi(LED_PORT, COMMAND_LED_CH);
					flag_korea = ~flag_korea;
					LCD_clearFast();
					if(flag_korea == TRUE) {
						previousMDA_degree = MDA_degree;
						previousMDA_minute = MDA_minute;
						MDA_degree = MDA_degree_korea;
						MDA_minute = MDA_minute_korea;
						LCD_position(0,0);
						LCD_string("Korea Peninsula ON");
						sprintf(buffer, "Time & Date: UTC+9");
						LCD_position(1,0);
						LCD_string(buffer);
					} else {
						MDA_degree = previousMDA_degree;
						MDA_minute = previousMDA_minute;
						LCD_position(0,0);
						LCD_string("World Map ON");
						sprintf(buffer, "Time & Date: UTC");
						LCD_position(1,0);
						LCD_string(buffer);
					}
					sprintf(buffer, "Updated MDA: %02dd%02dm", MDA_degree, MDA_minute);
					LCD_position(2,0);
					LCD_string(buffer);
					MagneticDeclinationAngle = DeclinationAngleOut(MDA_degree, MDA_minute);
					_delay_ms(5000); //To the next stage
					LCD_clearFast();
				} else if(flag_command == TRUE && key == '3') { //CMD3: Haversine/Vincenty
					key='X';
					flag_command = FALSE;
					cbi(LED_PORT, COMMAND_LED_CH);
					flag_vincenty=~flag_vincenty;
				}
				//Tracking mode CMD operating code stop
				
				/*
				//Tracking mode Only CMD3 operating code start
				if(flag_command == TRUE && key == '3') { //CMD3: Haversine/Vincenty
					key='X';
					flag_command = FALSE;
					cbi(LED_PORT, COMMAND_LED_CH);
					flag_vincenty=~flag_vincenty;
				}
				//Tracking mode Only CMD3 operating code start
				*/
				
				//Tracking mode CMD activating code start
				if(key=='#') {
					key='X';
					flag_command = ~flag_command;
					if(flag_command == TRUE) {
						sbi(LED_PORT, COMMAND_LED_CH);
						} else {
						cbi(LED_PORT, COMMAND_LED_CH);
					}
				}
				//Tracking mode CMD activating code stop
				
				_delay_ms(300); //Tracking mode key input delay and screen update period
			} //while(key!='*')
			//Before entering Initial state, {key, CLCD, CMD flag} reset code start
			key='X'; //Right after pressing '*'
			flag_command = FALSE;
			cbi(LED_PORT, COMMAND_LED_CH);
			LCD_clearFast();
			//Before entering Initial state, {key, CLCD, CMD flag} reset code stop
		//Initial state CMD activating code start
		} else if(key=='#') {
			key='X';
			flag_command = ~flag_command;
			if(flag_command == TRUE) {
				sbi(LED_PORT, COMMAND_LED_CH);
			} else {
				cbi(LED_PORT, COMMAND_LED_CH);
			}
		} //if(key=='*'), else if(key=='#')
		//Initial state CMD activating code stop
		_delay_ms(300); //Initial state key input delay and screen update period
	} //while(1)
} //int main()

void NegativeLED_Init(void) {
	sbi(LED_DDR, NEGATIVE_LED_CH);
}

void CommandLED_Init(void) {
	sbi(LED_DDR, COMMAND_LED_CH);
}

double LatitudeExtract(char *buffer_latitude, BOOL flag_negative) {
	double LatitudeValue=0;
	char buffer[4];
	for(unsigned char i=0; i<4; i++) {
		buffer[i]=buffer_latitude[i];
	}
	LatitudeValue=(double)atoi(buffer)*0.01;
	for(unsigned char i=0; i<3; i++) {
		buffer[i]=buffer_latitude[4+i];
	}
	buffer[3]='\0';
	LatitudeValue+=(double)atoi(buffer)*0.00001;
	if(flag_negative==TRUE) {
		LatitudeValue=(-1.0)*LatitudeValue;
	}
	
	return LatitudeValue;
}

double LongitudeExtract(char *buffer_longitude, BOOL flag_negative) {
	double LongitudeValue=0;
	char buffer[4];
	for(unsigned char i=0; i<4; i++) {
		buffer[i]=buffer_longitude[i];
	}
	LongitudeValue=(double)atoi(buffer)*0.1;
	for(unsigned char i=0; i<4; i++) {
		buffer[i]=buffer_longitude[4+i];
	}
	LongitudeValue+=(double)atoi(buffer)*0.00001;
	if(flag_negative==TRUE) {
		LongitudeValue=(-1.0)*LongitudeValue;
	}
	
	return LongitudeValue;
}

void DeclinationAngleUpdate(char *buffer_MDA, BOOL flag_negative) {
	char buffer[2];
	for(unsigned char i=0; i<2; i++) {
		buffer[i] = buffer_MDA[i];
	}
	MDA_degree = atoi(buffer);
	for(unsigned char i=0; i<2; i++) {
		buffer[i] = buffer_MDA[2+i];
	}
	MDA_minute = atoi(buffer);
	if(flag_negative==TRUE) {
		MDA_degree=(-1)*MDA_degree;
	}
	MagneticDeclinationAngle = DeclinationAngleOut(MDA_degree, MDA_minute);
}