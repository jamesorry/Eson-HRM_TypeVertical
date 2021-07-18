#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "Arduino.h"

#define DISPLAY_DEBUG   0
#define DISPLAY_LEFT	0
#define DISPLAY_RIGHT	1
#define DISPLAY_COLS	16

void Display_Init();
void Display(uint8_t lr, uint8_t x, uint8_t y, String str);
void Display(uint8_t lr, uint8_t x, uint8_t y, char *str);

#endif	//_DISPLAY_H_
