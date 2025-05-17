#include "pico/stdlib.h"
#include <stdio.h>
#include <cstring>
#include "fontData.h"
#define CHARX 8
#define CHARY 16
#define DATAX 144
#define DATAY 101

bool isValidCharacter(uint8_t c){
	bool valid = false;
	for(int chk = 0; chk < sizeof(fontMap); chk++){ 
		if(c == fontMap[chk]){ valid = true; } 
	}
	if(!valid) { return false; }
	return true;
}

bool getBit(int bitX, int bitY) {
	int index = bitY * DATAX + bitX;
	int byteindex = index / 8;
	int bitindex = index % 8;
	return (compressedFontData[byteindex] & (1 << bitindex)) != 0;
}

void drawRect(uint8_t* canvas, int x, int y, int width, int height, uint8_t shade){
	for (int ay = 0; ay < height; ay++) {
		for (int ax = 0; ax < width; ax++) {
			canvas[((y + ay) * 160) + (x + ax)] = shade;
		}
	}
}

void drawChar(uint8_t* canvas, int x, int y, char c){
    int cmIndex = 0;
    for (char cm : fontMap) {
        if (cm == c){ break; }
        cmIndex++;
    }

	int characterX = cmIndex % CHARY;
	int characterY = cmIndex / CHARY;

    for (int ay = 0; ay < CHARY; ay++) {
		for (int ax = 0; ax < CHARX; ax++) {
			if (getBit(ax + (characterX * CHARX) + characterX, ay + (characterY * CHARY) + characterY)) {
                canvas[((y + ay) * 160) + (x + ax)] = 0xFF;
			} else {
                canvas[((y + ay) * 160) + (x + ax)] = 0x00;
			}
		}
	}
}

void drawStr(uint8_t* canvas, int x, int y, const char* string){
	int strWidth = (CHARX + 2) * strlen(string);
	drawRect(canvas, x, y, strWidth + 4, CHARY + 2, 0x00);
	x+=3; y+=2;
    for(int i = 0; i < strlen(string); i++){
		drawChar(canvas, x, y, string[i]);
		x += CHARX + 2;
    }
}