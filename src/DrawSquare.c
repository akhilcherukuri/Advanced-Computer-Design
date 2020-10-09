/*
===============================================================================
 Name        : DrawSquare.c
 Author      : AKHIL CHERUKURI
 Version     : 1.0
 Copyright   : $(copyright)
 Description : Generate 2D Screen Saver of Rotating Squares
===============================================================================
*/

#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>



/* Be careful with the port number and location number, because

some of the location may not exist in that port. */

#define PORT_NUM            0


uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];


#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29



#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values

#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
// Self Defined Colors
#define PURPLE 0xCC33FF
#define YELLOW 0x00FFE0
#define BROWN 0xA52A2A

// One color for each set of rotation patterns
int x=0;
uint32_t a[100]={BLACK,GREEN,BLUE,RED,PURPLE,DARKBLUE,MAGENTA,LIGHTBLUE,WHITE,YELLOW,BROWN};

// Lamda value is in decimals
float LAMDA;


int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;


void spiwrite(uint8_t c)

{

 int pnum = 1;

 src_addr[0] = c;

 SSP_SSELToggle( pnum, 0 );

 SSPSend( pnum, (uint8_t *)src_addr, 1 );

 SSP_SSELToggle( pnum, 1 );

}



void writecommand(uint8_t c)

{

 LPC_GPIO0->FIOCLR |= (0x1<<21);

 spiwrite(c);

}



void writedata(uint8_t c)

{

 LPC_GPIO0->FIOSET |= (0x1<<21);

 spiwrite(c);

}



void writeword(uint16_t c)

{

 uint8_t d;

 d = c >> 8;

 writedata(d);

 d = c & 0xFF;

 writedata(d);

}



void write888(uint32_t color, uint32_t repeat)

{

 uint8_t red, green, blue;

 int i;

 red = (color >> 16);

 green = (color >> 8) & 0xFF;

 blue = color & 0xFF;

 for (i = 0; i< repeat; i++) {

  writedata(red);

  writedata(green);

  writedata(blue);

 }

}



void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)

{

 writecommand(ST7735_CASET);

 writeword(x0);

 writeword(x1);

 writecommand(ST7735_RASET);

 writeword(y0);

 writeword(y1);

}


void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t i;

 int16_t width, height;

 width = x1-x0+1;

 height = y1-y0+1;

 setAddrWindow(x0,y0,x1,y1);

 writecommand(ST7735_RAMWR);

 write888(color,width*height);

}



void lcddelay(int ms)

{

 int count = 24000;

 int i;

 for ( i = count*ms; i--; i > 0);

}



void lcd_init()

{

 int i;
 printf("LCD Demo Begins!!!\n");
 // Set pins P0.16, P0.21, P0.22 as output
 LPC_GPIO0->FIODIR |= (0x1<<16);

 LPC_GPIO0->FIODIR |= (0x1<<21);

 LPC_GPIO0->FIODIR |= (0x1<<22);

 // Hardware Reset Sequence
 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOCLR |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 // initialize buffers
 for ( i = 0; i < SSP_BUFSIZE; i++ )
 {

   src_addr[i] = 0;
   dest_addr[i] = 0;
 }

 // Take LCD display out of sleep mode
 writecommand(ST7735_SLPOUT);
 lcddelay(200);

 // Turn LCD display on
 writecommand(ST7735_DISPON);
 lcddelay(200);

}




void drawPixel(int16_t x, int16_t y, uint32_t color)

{

 if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))

 return;

 setAddrWindow(x, y, x + 1, y + 1);

 writecommand(ST7735_RAMWR);

 write888(color, 1);

}

// X Coordinate to Cartesian
int16_t xCartesian(int16_t x){
	x = x + (_width>>1);
	return x;
}

// Y Coordinate to Cartesian
int16_t yCartesian(int16_t y){
	y = (_height>>1) - y;
	return y;
}

// Rand() Function for X Coordinate
int randXline(){
	return ((rand() % _width) - (_width>>1));
}
// Rand() Function For Y Coordinate
int randYline(){
	return ((rand() % _height) - (_height>>1));
}




/*****************************************************************************


** Descriptions:        Draw line function

**

** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

** Returned value:        None

**

*****************************************************************************/


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

	// Cartesian Implementation
	x0 = xCartesian(x0);
	y0 = yCartesian(y0);
	x1 = xCartesian(x1);
	y1 = yCartesian(y1);

 int16_t slope = abs(y1 - y0) > abs(x1 - x0);

 if (slope) {

  swap(x0, y0);

  swap(x1, y1);

 }

 if (x0 > x1) {

  swap(x0, x1);

  swap(y0, y1);

 }

 int16_t dx, dy;

 dx = x1 - x0;

 dy = abs(y1 - y0);

 int16_t err = dx / 2;

 int16_t ystep;

 if (y0 < y1) {

  ystep = 1;

 }

 else {

  ystep = -1;

 }

 for (; x0 <= x1; x0++) {

  if (slope) {

   drawPixel(y0, x0, color);

  }

  else {

   drawPixel(x0, y0, color);

  }

  err -= dy;

  if (err < 0) {

   y0 += ystep;

   err += dx;

  }

 }

}

void drawRotatingSquare(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, int16_t i){
if(i==0)
{
	return;
}

	drawLine(x0, y0, x1, y1, a[x]);
	drawLine(x1, y1, x2, y2, a[x]);
	drawLine(x2, y2, x3, y3, a[x]);
	drawLine(x3, y3, x0, y0, a[x]);

	int p0, p1, p2, p3, p4, p5, p6, p7;

	p0 = x0 + LAMDA * (x1 - x0);
	p1 = y0 + LAMDA * (y1 - y0);
	p2 = x1 + LAMDA * (x2 - x1);
	p3 = y1 + LAMDA * (y2 - y1);
	p4 = x2 + LAMDA * (x3 - x2);
	p5 = y2 + LAMDA * (y3 - y2);
	p6 = x3 + LAMDA * (x0 - x3);
	p7 = y3 + LAMDA * (y0 - y3);

	drawLine(p0, p1, p2, p3, a[x]);
	drawLine(p2, p3, p4, p5, a[x]);
	drawLine(p4, p5, p6, p7, a[x]);
	drawLine(p6, p7, p0, p1, a[x]);

	lcddelay(250);

	drawRotatingSquare(p0, p1, p2, p3, p4, p5, p6, p7, i - 1);

}

/*

 Main Function main()

*/

int main (void)

{
	 uint32_t pnum = PORT_NUM;

	 pnum = 1 ;

	 if ( pnum == 1 )
		 SSP1Init();

	 else
		 puts("Port number is not correct");

	 lcd_init();

	 fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, WHITE);

	 	// User Defined LAMDA input
	 	char USER;
	 	printf("Do u want user inputed lamda value? (Y/N) : ");
	 	scanf("%c", &USER);
	 	if(USER == 'Y')
	 	{
	 	printf("Enter Lamda value : ");
	 	scanf("%f", &LAMDA);
	 	}
	 	else
	 	{
	 		LAMDA = 0.8;
	 	}
	 	// Continue to display each set of patterns without erasing the patterns
	    int w=0;
	 	while( w < 100){
	 				//RAND FUNCTION TO RADOMISED LOCATIONS
	 				int xr = randXline();
	 				int yr = randYline();
	 				//ROTATING SQUARE FUNCTION
	 				drawRotatingSquare(xr-40, yr+40, xr+40, yr+40, xr+40, yr-40, xr-40, yr-40, 12);
	 				//COLORS FOR EACH SET
	 				if(x==10){
	 					x=0;
	 				}
	 				x+=1;

	 	}

	  return 0;

}

