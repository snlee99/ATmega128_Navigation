#ifndef _MEGA128_4X3KEYMATRIX_H_
#define _MEGA128_4X3KEYMATRIX_H_

#define KEYMATRIX_DDR DDRA
#define KEYMATRIX_PORT PORTA
#define KEYMATRIX_PIN PINA

void keymatrix_init(void);
char keymatrix_value(void);

void keymatrix_init(void) {
	KEYMATRIX_DDR|=0x0F;
}

char keymatrix_value(void) {
	char keyBuff = 'X';
	unsigned char ROWS[] = {0x01, 0x02, 0x04, 0x08};
	unsigned char COLS[] = {0x10, 0x20, 0x40};
	
	KEYMATRIX_PORT = (0x80 & KEYMATRIX_PORT) | ROWS[0];
	_delay_us(10);
	if((KEYMATRIX_PIN & COLS[0]) == COLS[0]) {
		keyBuff = '1';
	} else if ((KEYMATRIX_PIN & COLS[1]) == COLS[1]) {
		keyBuff = '2';
	} else if ((KEYMATRIX_PIN & COLS[2]) == COLS[2]) {
		keyBuff = '3';
	}
	
	KEYMATRIX_PORT = (0x80 & KEYMATRIX_PORT) | ROWS[1];
	_delay_us(10);
	if((KEYMATRIX_PIN & COLS[0]) == COLS[0]) {
		keyBuff = '4';
	} else if ((KEYMATRIX_PIN & COLS[1]) == COLS[1]) {
		keyBuff = '5';
	} else if ((KEYMATRIX_PIN & COLS[2]) == COLS[2]) {
		keyBuff = '6';
	}
	
	KEYMATRIX_PORT = (0x80 & KEYMATRIX_PORT) | ROWS[2];
	_delay_us(10);
	if((KEYMATRIX_PIN & COLS[0]) == COLS[0]) {
		keyBuff = '7';
	} else if ((KEYMATRIX_PIN & COLS[1]) == COLS[1]) {
		keyBuff = '8';
	} else if ((KEYMATRIX_PIN & COLS[2]) == COLS[2]) {
		keyBuff = '9';
	}
	
	KEYMATRIX_PORT = (0x80 & KEYMATRIX_PORT) | ROWS[3];
	_delay_us(10);
	if((KEYMATRIX_PIN & COLS[0]) == COLS[0]) {
		keyBuff = '*';
	} else if ((KEYMATRIX_PIN & COLS[1]) == COLS[1]) {
		keyBuff = '0';
	} else if ((KEYMATRIX_PIN & COLS[2]) == COLS[2]) {
		keyBuff = '#';
	}
	
	return keyBuff;
}

#endif
