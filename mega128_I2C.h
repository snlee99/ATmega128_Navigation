#ifndef _MEGA128_I2C_H_
#define _MEGA128_I2C_H_

typedef  unsigned char  INT8;
typedef  unsigned int   INT16;

/** defines the data direction between master and slave in i2c_start(),i2c_rep_start() */
#define I2C_READ    1
#define I2C_WRITE   0
#define ACK 1
#define NACK 0

void i2c_init(void);
void i2c_start(void);
void i2c_write(INT8 data);
INT8 i2c_read(INT8 ackVal);
void i2c_stop(void);

void i2c_init(void) {
	DDRD |= 0x03; 
	PORTD |= 0x03;  // ** not busy = SCL,SDA High **
	TWSR = 0x00;   // TWPS(prescaler bit): 0
	TWBR = 0x20;   // SCL frequency is 100kHz @ 8MHz system frequency 
	TWCR = 0x04;   // enable TWI module
} 

void i2c_start(void) {
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

void i2c_write(INT8 data) {
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

INT8 i2c_read(INT8 ackVal) {
	TWCR = (1<<TWINT)|(1<<TWEN)|(ackVal<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	
	return TWDR;
}

void i2c_stop(void) {
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while (!(TWCR & (1<<TWSTO)));
	_delay_us(10);
}

#endif /* MEGA128_I2C_H_ */

