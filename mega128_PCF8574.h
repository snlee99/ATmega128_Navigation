#ifndef _MEGA128_PCF8574_H_
#define _MEGA128_PCF8574_H_

#include <util/delay.h>
#include "mega128_I2C.h"

#define PCF8574_ADDRBASE (0x27) //device slave address of PCF8574
//#define PCF8574_ADDRBASE (0x3F) //device slave address of PCF8574A

/* LCD - PCF8574 */
#define RS			0
#define RW			1
#define EN			2
#define BJT_BASE	3


#define CMD_CLEAR_DISPLAY                    0x01 // Clear All Display
#define CMD_RETURN_HOME                      0x02 // Cursor Position -> Return Home

#define CMD_ENTRY_MODE_SET_1           0x04 //   커서 좌측 이동, 화면이동 없음
#define CMD_ENTRY_MODE_SET_2           0x05 //   커서 좌측 이동, 화면 이동
#define CMD_ENTRY_MODE_SET_3           0x06 // * 커서 우측 이동, 화면이동 없음
#define CMD_ENTRY_MODE_SET_4           0x07 //   커서 우측 이동, 화면이동

#define CMD_DISPLAY_1                  0x08 //   화면 OFF,  커서 OFF,  커서 점멸 OFF
#define CMD_DISPLAY_2                  0x09 //   화면 OFF,  커서 OFF,  커서 점멸 ON
#define CMD_DISPLAY_3                  0x0A //   화면 OFF,  커서 ON ,  커서 점멸 OFF
#define CMD_DISPLAY_4                  0x0B //   화면 OFF,  커서 ON ,  커서 점멸 ON
#define CMD_DISPLAY_5                  0x0C //   화면 ON ,  커서 OFF,  커서 점멸 OFF
#define CMD_DISPLAY_6                  0x0D //   화면 ON ,  커서 OFF,  커서 점멸 ON
#define CMD_DISPLAY_7                  0x0E //   화면 ON ,  커서 ON ,  커서 점멸 OFF
#define CMD_DISPLAY_8                  0x0F // * 화면 ON ,  커서 ON ,  커서 점멸 ON

#define CMD_DISPLAY_SHIFT_1  			0x10 //   커서 선택,   커서 좌측 이동
#define CMD_DISPLAY_SHIFT_2				0x14 //   커서 선택,   커서 우측 이동
#define CMD_DISPLAY_SHIFT_3				0x18 //   화면 선택,   커서 좌측 이동
#define CMD_DISPLAY_SHIFT_4				0x1C //   화면 선택,   커서 우측 이동

#define CMD_FUNCTION_SET_1              0x20 //   4비트,   화면1행,    5x8  Font
#define CMD_FUNCTION_SET_2              0x24 // * 4비트,   화면1행,    5x11 Font
#define CMD_FUNCTION_SET_3              0x28 // * 4비트,   화면2행,    5x8  Font
#define CMD_FUNCTION_SET_4              0x2C //   4비트,   화면2행,    5x11 Font
#define CMD_FUNCTION_SET_5              0x30 //   8비트,   화면1행,    5x8  Font
#define CMD_FUNCTION_SET_6              0x34 // * 8비트,   화면1행,    5x11 Font
#define CMD_FUNCTION_SET_7              0x38 // * 8비트,   화면2행,    5x8  Font
#define CMD_FUNCTION_SET_8              0x3C //   8비트,   화면2행,    5x11 Font

void LCD_clear(void);
void Initialize_LCD(void);
void LCD_position(INT8 row, INT8 col);
void LCD_string(char *string);
void I2C_LCD_data(INT8 data);
void I2C_LCD_command(INT8 command);
void I2C_LCD_command_8(INT8 command);
void LCD_rowClear(uint8_t row);

void I2C_LCD_command_8(INT8 command) {// write a command(instruction) to text LCD
	// 8-bit interface
	uint8_t T_buf[2]; // transmit buffer
	T_buf[0] = (command & 0xF0)|(0<<BJT_BASE)|(1<<EN)|(0<<RW)|(0<<RS); // high 4 bit, base = high (Backlight on), EN = 1, RW = 0(write), RS = 0
	T_buf[1] = T_buf[0] & ~(1<<EN);       // EN = 0
	i2c_start();            // transmit START condition
	i2c_write((PCF8574_ADDRBASE<<1)|I2C_WRITE);  // address PCF8574 to write
	i2c_write(T_buf[0]);
	i2c_write(T_buf[1]);
	i2c_stop();             // transmit STOP condition
}

void I2C_LCD_command(INT8 command) {// write a command(instruction) to text LCD
	// 4-bit interface
	uint8_t T_buf[4]; // transmit buffer
	T_buf[0] = (command & 0xF0)|(1<<BJT_BASE)|(1<<EN)|(0<<RW)|(0<<RS); // high 4 bit, base = high (Backlight on), EN = 1, RW = 0(write), RS = 0
	T_buf[1] = T_buf[0] & ~(1<<EN);        // EN = 0
	
	T_buf[2] = ((command & 0x0F) << 4)|(1<<BJT_BASE)|(1<<EN)|(0<<RW)|(0<<RS); // low 4 bit , base = high (Backlight on), EN = 1, RW = 0(write), RS = 0
	T_buf[3] = T_buf[2] & ~(1<<EN);       // EN = 0
	
	i2c_start();            // transmit START condition
	i2c_write((PCF8574_ADDRBASE<<1)|I2C_WRITE);  // address PCF8574 to write
	i2c_write(T_buf[0]);
	i2c_write(T_buf[1]);
	i2c_write(T_buf[2]);
	i2c_write(T_buf[3]);
	i2c_stop();             // transmit STOP condition
}

void I2C_LCD_data(INT8 data) {// display a character on text LCD
	uint8_t T_buf[4]; // transmit buffer
	T_buf[0] = (data & 0xF0)|(1<<BJT_BASE)|(1<<EN)|(0<<RW)|(1<<RS); // high 4 bit, base = high (Backlight on), EN = 1, RW = 0(write), RS = 1(data)
	T_buf[1] = T_buf[0] & ~(1<<EN);       // EN = 0
	
	T_buf[2] = ((data & 0x0F) << 4)|(1<<BJT_BASE)|(1<<EN)|(0<<RW)|(1<<RS); // low 4 bit , base = high (Backlight on), EN = 1, RW = 0(write), RS = 1(data)
	T_buf[3] = T_buf[2] & ~(1<<EN);        // EN = 0
	
	i2c_start();            // transmit START condition
	i2c_write((PCF8574_ADDRBASE<<1)|I2C_WRITE);  // address PCF8574 to write
	i2c_write(T_buf[0]);
	i2c_write(T_buf[1]);
	i2c_write(T_buf[2]);
	i2c_write(T_buf[3]);
	i2c_stop();             // transmit STOP condition
}

void LCD_string(char *string) {// display a string on LCD
	while (*string != '\0')   // display string
	{
		I2C_LCD_data(*string);
		string++;
	}
}

void LCD_position(INT8 row, INT8 col) {
	/*1602 LCD*/
	//I2C_LCD_command(0x80|(row*0x40+col));
	/*2004 LCD*/
	uint8_t position[]={0x80, 0xC0, 0x94, 0xD4};
	I2C_LCD_command(position[row]+col);
}

void Initialize_LCD(void) {// initialize text LCD module
	_delay_ms(50);
	// 8-bit mode
	I2C_LCD_command_8(CMD_FUNCTION_SET_5);	//   8비트,   화면1행,    5x8  Font
	_delay_ms(5);
	I2C_LCD_command_8(CMD_FUNCTION_SET_5);
	_delay_ms(1);
	I2C_LCD_command_8(CMD_FUNCTION_SET_5);
	I2C_LCD_command_8(CMD_FUNCTION_SET_1);	//   4비트,   화면1행,    5x8  Font

	// 4-bit mode
	I2C_LCD_command(CMD_FUNCTION_SET_3);		// * 4비트,   화면2행,    5x8  Font
	I2C_LCD_command(CMD_DISPLAY_5);			//   화면 ON ,  커서 OFF,  커서 점멸 OFF
	I2C_LCD_command(CMD_CLEAR_DISPLAY);				// clear display
	I2C_LCD_command(CMD_ENTRY_MODE_SET_3);	// * 커서 우측 이동, 화면이동 없음
	_delay_ms(3);
}

void LCD_clear(void) {
	I2C_LCD_command(0x01);
	_delay_ms(5);
}

void LCD_rowClear(uint8_t row) {
	LCD_position(row,0);
	LCD_string("                    ");
}

void LCD_clearFast(void) {
	LCD_rowClear(0);
	LCD_rowClear(1);
	LCD_rowClear(2);
	LCD_rowClear(3);
}

#endif /* MEGA128_PCF8754_H_ */