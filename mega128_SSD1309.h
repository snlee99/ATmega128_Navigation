#ifndef _MEGA128_SSD1309_H_
#define _MEGA128_SSD1309_H_

#define SSD1309_ADDRESS 0x3C
#define SSD1309_MODE_CMD 0x00
#define SSD1309_MODE_DATA 0x40
#define SSD1309_CMD_SET_COLUMN_ADDRESS 0x21
#define SSD1309_CMD_SET_PAGE_ADDRESS 0x22
#define SSD1309_CMD_SET_DISPLAY_START_LINE 0x40
#define SSD1309_CMD_SET_PAGE_START_ADDRESS 0xB0
#define SSD1309_WIDTH 128
#define SSD1309_HEIGHT 64

#define KOREA_PENINSULA_LAT_MIN 33.0
#define KOREA_PENINSULA_LAT_MAX 43.0
#define KOREA_PENINSULA_LON_MIN 124.0
#define KOREA_PENINSULA_LON_MAX 131.0

#include <string.h>
#include "mega128_I2C.h"

typedef struct tagPixel {
	uint8_t x;
	uint8_t y;	
} Pixel;

const uint8_t Init_Sequence[] = {
	0xAE,		//디스플레이 OFF
	0xD5, 0x80,	//디스플레이 주파수 설정
	0xA8, 0x3F,	//컬럼 수 설정
	0xD3, 0x00,	//디스플레이 시작 라인 설정
	0x40,		//디스플레이 시작 위치 설정
	0x8D, 0x14,	//전원 영역 설정
	0x20, 0x00,	//주소 모드: 수평
	0xA1,		//세그먼트 리맵: 컬럼 0이 왼쪽
	0xC8,		//컬럼 리맵: 상하대칭 없음
	0xDA, 0x12,	//컴플레이션 제어: 양극성 설정
	0x81, 0xCF,	//컨트래스 설정
	0xD9, 0xF1,	//선행 주기 설정
	0xD8, 0x40,	//VCOMH 레벨 설정
	0xA4,		//디스플레이 출력 설정: GDRAM에 기록
	0xA6,		//출력 반전 모드 비활성화
	0xAF		//디스플레이 ON
};

const uint8_t worldMap[SSD1309_WIDTH*SSD1309_HEIGHT/8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x30, 0x30,
	0x38, 0x18, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80,
	0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x02,
	0x00, 0x00, 0x00, 0x00, 0x18, 0x7C, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFA, 0xF0, 0x20,
	0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x20, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0x60, 0x38, 0x78, 0x3C, 0x7C, 0x7C,
	0x7C, 0x7C, 0x7E, 0x7E, 0xFE, 0xFE, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xE3, 0xC3, 0xE0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF,
	0x47, 0x07, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x1F, 0x3F, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F,
	0x0C, 0x0C, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xE0,
	0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0xC1, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x38, 0x7C,
	0x98, 0x10, 0x10, 0x10, 0x10, 0x1B, 0x3F, 0x3F, 0x0F, 0x1F, 0x0F, 0x9F, 0xBF, 0xFF, 0xFF, 0x7F,
	0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x39, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7C, 0x78, 0x70, 0x21,
	0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x0F, 0x3F, 0x3F, 0x7F, 0x3F, 0x3F, 0x0F, 0x0E, 0x0E, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x63, 0xFF, 0xCF, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xC0,
	0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x07, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x03, 0x03, 0x03, 0x03,
	0x07, 0x07, 0x0F, 0x0F, 0x0F, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x70, 0x78,
	0x78, 0x78, 0x38, 0x38, 0x18, 0x08, 0x08, 0x00, 0x60, 0x60, 0x70, 0x78, 0x78, 0x78, 0x78, 0x78,
	0x78, 0x78, 0x78, 0x78, 0x60, 0x60, 0x60, 0x60, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x60, 0x70, 0x70, 0x70, 0x78, 0x78, 0x78, 0x7C, 0x7C, 0x7C,
	0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3C, 0x2E, 0x06, 0x00, 0x02,
	0x40, 0x60, 0x70, 0x7C, 0x7C, 0x7E, 0x7E, 0x7E, 0x78, 0x78, 0x7C, 0x7C, 0x7C, 0x7E, 0x7F, 0x7F,
	0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x3F, 0x1F, 0x1F, 0x03, 0x03, 0x03, 0x41, 0x60, 0x70, 0x78
};

const uint8_t koreaPeninsula [SSD1309_WIDTH*SSD1309_HEIGHT/8] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x0F, 0x07, 0x07,
	0x07, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0x31, 0x01, 0x01, 0x05, 0x01, 0x00, 0x00, 0x01, 0x03, 0x03, 0x0F, 0x1F, 0x3F, 0x3F,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x01, 0x19, 0x11, 0x01, 0x01, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03,
	0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x07, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x80, 0x00, 0x06, 0x00,
	0x00, 0x00, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07, 0x27, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x01, 0x01, 0x01, 0x03,
	0x01, 0x03, 0x07, 0x0F, 0x71, 0x31, 0x31, 0x70, 0x00, 0x00, 0x0F, 0x03, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0F, 0x1F, 0x1F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xE6, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC1, 0xC3, 0xC3, 0xC3, 0xC3, 0xC1, 0xC0, 0xC0, 0xF8,
	0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xE3,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xEC, 0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C,
	0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x08, 0xC0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x7F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xC0, 0xC0, 0xC0, 0xC2, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xF8, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xEE, 0xC4, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0x00, 0x60, 0x40, 0x40,
	0x00, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
	0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x7C, 0x78,
	0x70, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xC0, 0xC0, 0xC0,
	0xC0, 0xF8, 0xF8, 0x30, 0x20, 0x00, 0x80, 0x00, 0x80, 0x80, 0x80, 0xD0, 0xD0, 0xD0, 0xC0, 0xFF
};

volatile uint8_t frameBuffer[SSD1309_WIDTH*SSD1309_HEIGHT/8];
//memmove(frameBuffer, worldMap, sizeof(worldMap));

void SSD1309_SendCommand(uint8_t cmd);
void SSD1309_Init();
void SSD1309_SetPixel(uint8_t x, uint8_t y);
void SSD1309_ClearPixel(uint8_t x, uint8_t y);
void SSD1309_FillScreen(void);
void SSD1309_ClearScreen(void);
void SSD1309_SetHorizon(uint8_t y);
void SSD1309_SetVertical(uint8_t x);
void SSD1309_UpdateScreen(void);

void SSD1309_SendCommand(uint8_t cmd) {
	i2c_start();
	i2c_write(SSD1309_ADDRESS<<1|I2C_WRITE);
	i2c_write(SSD1309_MODE_CMD);
	i2c_write(cmd);
	i2c_stop();
}

void SSD1309_Init() {
	for(uint8_t i = 0; i < sizeof(Init_Sequence); i++) {
		SSD1309_SendCommand(Init_Sequence[i]);
	}
}

void SSD1309_SetPixel(uint8_t x, uint8_t y) {
	frameBuffer[x+(y/8)*SSD1309_WIDTH] |= (1<<(y%8));
}

void SSD1309_ClearPixel(uint8_t x, uint8_t y) {
	frameBuffer[x+(y/8)*SSD1309_WIDTH] &= ~(1<<(y%8));
}

void SSD1309_FillScreen(void) {
	for(uint16_t i = 0; i < sizeof(frameBuffer); i++) {
		frameBuffer[i] = 0xFF;
	}
}

void SSD1309_ClearScreen(void) {
	for(uint16_t i = 0; i < sizeof(frameBuffer); i++) {
		frameBuffer[i] = 0x00;
	}
}

void SSD1309_SetHorizon(uint8_t y) {
	for(uint8_t i=0; i<SSD1309_WIDTH; i++) {
		SSD1309_SetPixel(i, y);
	}
}

void SSD1309_SetVertical(uint8_t x) {
	for(uint8_t i=0; i<SSD1309_HEIGHT; i++) {
		SSD1309_SetPixel(x, i);
	}
}

void SSD1309_UpdateScreen(void) {
	for(uint8_t page = 0; page < SSD1309_HEIGHT/8; page++) {
		SSD1309_SendCommand(SSD1309_CMD_SET_PAGE_START_ADDRESS + page);
		SSD1309_SendCommand(0x00);
		SSD1309_SendCommand(0x10);
		i2c_start();
		i2c_write(SSD1309_ADDRESS<<1|I2C_WRITE);
		i2c_write(SSD1309_MODE_DATA);
		for(uint8_t column = 0; column < SSD1309_WIDTH; column++) {
			i2c_write(frameBuffer[column + page*SSD1309_WIDTH]);
		}
		i2c_stop();
	}
}

#endif