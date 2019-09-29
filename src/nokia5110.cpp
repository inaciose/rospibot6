//
// nokia 5110 lcd c driver
// based on work of vellamy
// http://vellamy.blogspot.com/p/blog-page.html
//

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <bcm2835.h>

#include "nokia5110_def.h"

unsigned char lcdRSTpin = 0;
unsigned char lcdCEpin = 0;
unsigned char lcdDCpin = 0;
unsigned char lcdDINpin = 0;
unsigned char lcdCLKpin = 0;
unsigned char lcdLIGHTpin = 0;

int lcdScrollPos = -10;
int lcdLin = 0;
int lcdPos = 0;

// bitbang serial shift out on select GPIO pin. Data rate is defined by CPU clk speed and CLKCONST_2.
// Calibrate these value for your need on target platform.
void lcdShiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
        uint8_t i;
        uint32_t j;

        for (i = 0; i < 8; i++)  {
                if (bitOrder == LCD_LSBFIRST)
                        bcm2835_gpio_write(dataPin, !!(val & (1 << i)));
                else
                        bcm2835_gpio_write(dataPin, !!(val & (1 << (7 - i))));

                bcm2835_gpio_write(clockPin, HIGH);
                for (j = LCD_CLKCONST_2; j > 0; j--); // clock speed, anyone? (LCD Max CLK input: 4MHz)
                bcm2835_gpio_write(clockPin, LOW);
        }
}

//There are two memory banks in the LCD, data/RAM and commands.
//This function sets the DC pin high or low depending, and then sends the data byte
void lcdWrite(uint8_t dc, uint8_t data) {
	bcm2835_gpio_write(lcdDCpin, dc);//Tell the LCD that we are writing either to data or a command
	//Send the data
	bcm2835_gpio_write(lcdCEpin, LOW);
	lcdShiftOut(lcdDINpin, lcdCLKpin, LCD_MSBFIRST, data);
	bcm2835_gpio_write(lcdCEpin, HIGH);
}

//Switch on/off the back light. "state" should be true or false
void lcdBackLight (bool state) {
	bcm2835_gpio_write(lcdLIGHTpin, state);
}

/**
 * gotoXY routine to position cursor
 * x - range: 0 to 84
 * y - range: 0 to 5
 */
void lcdGotoXY(int x, int y) {
	lcdWrite( 0, 0x80 | x);  // Column.
	lcdWrite( 0, 0x40 | y);  // Row.
}

void lcdCharacter(char character) {
	int index;
	lcdWrite(LCD_DATA, 0x00);
	for (index = 0; index < 5; index++)   {
		lcdWrite(LCD_DATA, ASCII[character - 0x20][index]);
	}
	lcdWrite(LCD_DATA, 0x00);

	if (lcdPos >= 504) lcdPos = 0;
	else lcdPos += 6;
	lcdLin=(lcdPos / LCD_X);
	if (lcdLin == 6 ) lcdLin = 0;
}

void lcdClear(void) {
	int index;
	for (index = 0; index < LCD_X * LCD_Y / 8; index++) {
		lcdWrite(LCD_DATA, 0x00);
	}
	lcdPos=0;
	lcdGotoXY(0, 0);
}

void lcdCreate(unsigned char rst, unsigned char ce, unsigned char dc, unsigned char din, unsigned char clk, unsigned char bl) {

	// set nokia5110 pins
	lcdRSTpin = rst;
	lcdCEpin = ce;
	lcdDCpin = dc;
	lcdDINpin = din;
	lcdCLKpin = clk;
	lcdLIGHTpin = bl;

	// set arm pins modes
        bcm2835_gpio_fsel(lcdCEpin,  BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(lcdRSTpin,BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(lcdDCpin,   BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(lcdDINpin, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(lcdCLKpin, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(lcdLIGHTpin,   BCM2835_GPIO_FSEL_OUTP);

}
void lcdReset(void) {
        bcm2835_gpio_write(lcdCEpin, LOW);
        bcm2835_gpio_write(lcdRSTpin, LOW);
        bcm2835_delay(500);
        bcm2835_gpio_write(lcdRSTpin, HIGH);
}

// sends the commands to the PCD8544
void lcdInit(void) {

	lcdReset();

	lcdWrite(LCD_COMMAND, 0x21);//Tell LCD that extended commands follow
	lcdWrite(LCD_COMMAND, 0xBf);//Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
	lcdWrite(LCD_COMMAND, 0x04);// Set Temp coefficent. //0x04
	lcdWrite(LCD_COMMAND, 0x14);// LCD bias mode 1:48: Try 0x13 or 0x14

	lcdWrite(LCD_COMMAND, 0x20);//We must send 0x20 before modifying the display control mode
	lcdWrite(LCD_COMMAND, 0x0C);//Set display control, normal mode. 0x0D for inverse
}

void lcdString(char *characters) {
	while (*characters) {
		lcdCharacter(*characters++);
	}
	lcdWrite(0, 0x40 | lcdLin);  // Row.
}

//Do LCDString() and It passes the next caracter to the next row
void lcdStringln(char *characters) {
	while (*characters) {
        	lcdCharacter(*characters++);
	}
	lcdWrite(0, 0x80 | 0);  // Column.
	lcdWrite(0, 0x40 |(lcdLin + 1));  // Row.  ?
}

// TO REVISE
/*
void lcdScroll(char message[]) {
	int i;
	unsigned int message_length=0;
	message_length = strlen(message);
	for (i = lcdScrollPos; i < lcdScrollPos + 11; i++)   {
		if ((i >= message_length) || (i < 0))     {
			lcdCharacter(' ');
		} else {
			lcdCharacter(message[i]);
		}
	}
	lcdScrollPos++;
	if ((lcdScrollPos >= message_length) && (lcdScrollPos > 0)) {
		lcdScrollPos = -10;
	}
}
*/
