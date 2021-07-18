#include "Arduino.h"
#include "Display.h"

#include <Wire.h>  // Arduino IDE 內建
// LCD I2C Library，從這裡可以下載：
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C *lcd[2] ;

extern HardwareSerial *cmd_port;
void Display_Init() 
{
	for(int i=0; i<2; i++)
	{	//Addr 0x27~0x20, A0~A2 pull high
		lcd[i] = new LiquidCrystal_I2C(0x27-i, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);	// 設定 LCD I2C 位址
		lcd[i]->begin(DISPLAY_COLS, 2);      // 初始化 LCD，一行 16 的字元，共 2 行，預設開啟背光
		lcd[i]->backlight();
		delay(100);
#if DISPLAY_DEBUG
		cmd_port->println("LCD ptr" + String(i) + ": " + String((uint32_t) lcd[i]));
#endif
	}
}

void Display(uint8_t lr, uint8_t x, uint8_t y, String str) 
{
	char buffer[21];
	str.toCharArray(buffer, 20);
	Display(lr, x, y, buffer);
}

void Display(uint8_t lr, uint8_t x, uint8_t y, char *str) 
{
#if DISPLAY_DEBUG
	cmd_port->println("LR: " + String(lr) + ", (" + String(x) + ", " + String(y) + "), " + str + " Len:" + String(strlen(str)));
#endif
	//for(int i=strlen(str)+x; i<DISPLAY_COLS; i++)
	//	str[i] = 0x20;
	//str[DISPLAY_COLS] = 0x00;
	lcd[lr]->setCursor(x, y);
	lcd[lr]->write(str);
	delay(50);
}
