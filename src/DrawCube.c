/*
===============================================================================
 Name        : 2019F_LAB3_014525420.c
 Author      : AKHIL CHERUKURI
 Version     : FINAL
 Copyright   : $(copyright)
 Description : Lab LPC-ARM Architecture 3D Graphics Processing Engine Design
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
#include <time.h>

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

// Defining Colors

#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF

// Self Defined Colors
#define YELLOW 0xFFFF00
#define BROWN 0xA52A2A
#define BLUE1 0x3999FF
#define ORANGE1 0xFFA845
#define GREEN1 0x93FF60
#define GREY1 0x7A7C7B


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
 //int16_t i;
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
 printf("Lab LPC-ARM Architecture Based 3D Graphics Processing Engine Design\n");
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

// Coordinate to Cartesian
int16_t xCartesian(int16_t x){
	x = x + (_width>>1);
	return x;
}

int16_t yCartesian(int16_t y){
	y = (_height>>1) - y;
	return y;
}

void drawPixel(int16_t x, int16_t y, uint32_t color)
{
	// Cartesian Implementation
	x=xCartesian(x); y=yCartesian(y);

 if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
 return;
 setAddrWindow(x, y, x + 1, y + 1);
 writecommand(ST7735_RAMWR);
 write888(color, 1);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
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

// 3D ENGINE

float Lambda3D(float Zi,float Zs)
{
	float lambda;
	lambda = -Zi/(Zs-Zi);
	return lambda;
}

typedef struct{
	float_t x_value; float_t y_value; float_t z_value;
}Pts3D;

typedef struct{
	float_t x; float_t y;
}Pts2D;

Pts3D ShadowPoint3D(Pts3D Pi, Pts3D Ps, float lambda)
{
	Pts3D pt;
	pt.x_value = (Pi.x_value + (lambda*(Ps.x_value-Pi.x_value)));
	pt.y_value = (Pi.y_value + (lambda*(Ps.y_value-Pi.y_value)));
	pt.z_value = (Pi.z_value + (lambda*(Ps.z_value-Pi.z_value)));
	return pt;
}

Pts2D get3DTransform(Pts3D Pi)
{
	float_t Xe=130, Ye=130, Ze=130;
	float_t Rho=sqrt(pow(Xe,2)+pow(Ye,2)+pow(Xe,2));
	float_t D_focal=100;

	typedef struct{
		float X; float Y; float Z;
	}pworld;

	typedef struct{
		float X; float Y; float Z;

	}pviewer;

	typedef struct{
		float X; float Y;
	}pperspective;

	pworld world;
	pviewer viewer;
	pperspective perspective;

	world.X = Pi.x_value;
	world.Y = Pi.y_value;
	world.Z = Pi.z_value;

	float sPheta = Ye/sqrt(pow(Xe,2)+pow(Ye,2));
	float cPheta = Xe/sqrt(pow(Xe,2)+pow(Ye,2));
	float sPhi = sqrt(pow(Xe,2)+pow(Ye,2))/Rho;
	float cPhi = Ze/Rho;

	viewer.X=-sPheta*world.X+cPheta*world.Y;
	viewer.Y = -cPheta * cPhi * world.X - cPhi * sPheta * world.Y + sPhi * world.Z;
	viewer.Z = -sPhi * cPheta * world.X - sPhi * cPheta * world.Y-cPheta * world.Z + Rho;

	perspective.X=D_focal*viewer.X/viewer.Z;
	perspective.Y=D_focal*viewer.Y/viewer.Z;

	Pts2D pt;
	pt.x=perspective.X;
	pt.y=perspective.Y;
	return pt;

}

int getDiffuseColor(Pts3D Pi)
{
	uint32_t print_diffuse_color;
	Pts3D Ps = {40,60,120};
	int diff_red, diff_green, diff_blue;
	float new_red, new_green, new_blue, r1=0.8, r2=0.0, r3=0.0, scaling = 4000;

	float_t temp = ((Ps.z_value - Pi.z_value)/sqrt(pow((Ps.x_value - Pi.x_value),2) + pow((Ps.y_value - Pi.y_value),2) + pow((Ps.z_value - Pi.z_value),2))) / (pow((Ps.x_value - Pi.x_value),2) + pow((Ps.y_value - Pi.y_value),2) + pow((Ps.z_value - Pi.z_value),2));

	temp *= scaling;

	diff_red = r1 * temp * 255;
	diff_green = r2 * temp;
	diff_blue  = r3 * temp;
	new_red = (diff_red << 16);
	new_green = (diff_green << 8);
	new_blue = (diff_blue);
	print_diffuse_color = new_red + new_green +new_blue;
	return print_diffuse_color;
}

void drawCube(){
#define UpperBD 52
#define NumOfPts 10
#define Pi 3.1415926
typedef struct{
		float X[UpperBD]; float Y[UpperBD]; float Z[UpperBD];

	}pworld;
typedef struct{
		float X[UpperBD]; float Y[UpperBD]; float Z[UpperBD];

	}pviewer;
typedef struct{
		float X[UpperBD]; float Y[UpperBD];
	}pperspective;

// EYE VIEW(130,130,130),

float_t Xe=130, Ye=130, Ze=130;

float_t Rho=sqrt(pow(Xe,2)+pow(Ye,2)+pow(Xe,2));
float_t D_focal=100;
float_t L1,L2,L3,L4;

pworld WCS;
pviewer V;
pperspective P;

// X-Y-Z World Coordinate System

// Origin
WCS.X[0]=0.0; WCS.Y[0]=0.0; WCS.Z[0]=0.0;
WCS.X[1]=50.0; WCS.Y[1]=0.0; WCS.Z[1]=0.0;
WCS.X[2]=0.0; WCS.Y[2]=50.0; WCS.Z[2]=0.0;
WCS.X[3]=0.0; WCS.Y[3]=0.0;	WCS.Z[3]=50.0;

// Elevate Cube along Z_w axis by 10

WCS.X[4]=50.0; WCS.Y[4]=0; WCS.Z[4]=10.0;
WCS.X[5]=0.0; WCS.Y[5]=50.0; WCS.Z[5]=10.0;
WCS.X[6]=0.0; WCS.Y[6]=0.0; WCS.Z[6]=60.0;
WCS.X[7]=50.0; WCS.Y[7]=0.0; WCS.Z[7]=60.0;
WCS.X[8]=50.0; WCS.Y[8]=50.0; WCS.Z[8]=10.0;
WCS.X[9]=0.0; WCS.Y[9]=50.0; WCS.Z[9]=60.0;
WCS.X[10]=50.0; WCS.Y[10]=50.0; WCS.Z[10]=60.0;

// POINT OF LIGHT SOURCE PS(40, 60, 120)

WCS.X[11]=40.0; WCS.Y[11]=60.0; WCS.Z[11]=120.0;


Pts3D Ps;
Ps.x_value = WCS.X[11]; Ps.y_value = WCS.Y[11]; Ps.z_value = WCS.Z[11];

Pts3D P1;
P1.x_value = WCS.X[6]; P1.y_value = WCS.Y[6]; P1.z_value = WCS.Z[6];

Pts3D P2;
P2.x_value = WCS.X[7]; P2.y_value = WCS.Y[7]; P2.z_value = WCS.Z[7];

Pts3D P3;
P3.x_value = WCS.X[9]; P3.y_value = WCS.Y[9]; P3.z_value = WCS.Z[9];

Pts3D P4;
P4.x_value = WCS.X[10]; P4.y_value = WCS.Y[10]; P4.z_value = WCS.Z[10];


// Shadow Points

Pts3D S1,S2,S3,S4;
L1 = Lambda3D(WCS.Z[6],WCS.Z[11]);
L2 = Lambda3D(WCS.Z[7],WCS.Z[11]);
L3 = Lambda3D(WCS.Z[9],WCS.Z[11]);
L4 = Lambda3D(WCS.Z[10],WCS.Z[11]);
S1 = ShadowPoint3D(P1,Ps,L1);
S2 = ShadowPoint3D(P2,Ps,L2);
S3 = ShadowPoint3D(P3,Ps,L3);
S4 = ShadowPoint3D(P4,Ps,L4);

WCS.X[12]=S1.x_value;		WCS.Y[12]=S1.y_value;		WCS.Z[12]=S1.z_value; //S1
WCS.X[13]=S2.x_value;		WCS.Y[13]=S2.y_value;		WCS.Z[13]=S2.z_value; //S2
WCS.X[14]=S3.x_value;		WCS.Y[14]=S3.y_value;		WCS.Z[14]=S3.z_value; //S3
WCS.X[15]=S4.x_value;		WCS.Y[15]=S4.y_value;		WCS.Z[15]=S4.z_value; //S4


float sPheta = Ye/sqrt(pow(Xe,2)+pow(Ye,2));
float cPheta = Xe/sqrt(pow(Xe,2)+pow(Ye,2));
float sPhi = sqrt(pow(Xe,2)+pow(Ye,2))/Rho;
float cPhi = Ze/Rho;

// WORLD TO VIEWER TRANSFROM

for(int i=0;i<=UpperBD;i++)
{
	V.X[i] = -sPheta*WCS.X[i]+cPheta*WCS.Y[i];
	V.Y[i] = -cPheta * cPhi * WCS.X[i]- cPhi * sPheta * WCS.Y[i]+ sPhi * WCS.Z[i];
	V.Z[i] = -sPhi * cPheta * WCS.X[i]- sPhi * cPheta * WCS.Y[i]-cPheta * WCS.Z[i] + Rho;
}

for(int i=0;i<=UpperBD;i++)
{
	P.X[i]=D_focal*V.X[i]/V.Z[i];
	P.Y[i]=D_focal*V.Y[i]/V.Z[i];

}

//CUBE SHADOWS

// SHADOW DRAWLINES
drawLine(P.X[12],P.Y[12],P.X[13],P.Y[13],GREY1);
drawLine(P.X[13],P.Y[13],P.X[15],P.Y[15],GREY1);
drawLine(P.X[14],P.Y[14],P.X[15],P.Y[15],GREY1);
drawLine(P.X[14],P.Y[14],P.X[12],P.Y[12],GREY1);
//printf("ShadowPoint 12 : %f %f %f \n",S1.x_value,S1.y_value,S1.z_value);
//printf("ShadowPoint 13 : %f %f %f \n",S2.x_value,S2.y_value,S2.z_value);
//printf("ShadowPoint 14 : %f %f %f \n",S3.x_value,S3.y_value,S3.z_value);
//printf("ShadowPoint 15 : %f %f %f \n",S4.x_value,S4.y_value,S4.z_value);

//SHADOW FILL
for(int x=-40;x<=60;x++)
	for(int y=-60;y<=40;y++)
	{
		Pts3D pt; Pts2D pt_new;
		pt.x_value=x; pt.y_value=y; pt.z_value=0;
		pt_new = get3DTransform(pt);
		drawPixel(pt_new.x,pt_new.y,GREY1);
	}

// LIGHT RAYS DRAWLINE BACK
drawLine(P.X[12],P.Y[12],P.X[11],P.Y[11],GREY1);

// CUBE DRAWLINES
drawLine(P.X[7],P.Y[7],P.X[4],P.Y[4],BLACK);
drawLine(P.X[7],P.Y[7],P.X[6],P.Y[6],BLACK);
drawLine(P.X[7],P.Y[7],P.X[10],P.Y[10],BLACK);
drawLine(P.X[8],P.Y[8],P.X[4],P.Y[4],BLACK);
drawLine(P.X[8],P.Y[8],P.X[5],P.Y[5],BLACK);
drawLine(P.X[8],P.Y[8],P.X[10],P.Y[10],BLACK);
drawLine(P.X[9],P.Y[9],P.X[6],P.Y[6],BLACK);
drawLine(P.X[9],P.Y[9],P.X[5],P.Y[5],BLACK);
drawLine(P.X[9],P.Y[9],P.X[10],P.Y[10],BLACK);
//RED TOP SIDE FILL
float diff_color;
for(int y=0;y<=50;y++)
	for(int x=0;x<=50;x++)
	{
		Pts3D pt; Pts2D pt2d;
		pt.x_value=x; pt.y_value=y; pt.z_value=60;
		pt2d = get3DTransform(pt);
		// RED DIFFUSE
		diff_color = getDiffuseColor(pt);
		drawPixel(pt2d.x,pt2d.y,diff_color);
	}
//GREEN FRONT SIDE FILL
for(int y=0;y<=50;y++)
	for(int z=60;z>=10;z--)
	{
		Pts3D pt; Pts2D pt2d;
		pt.x_value=50; pt.y_value=y; pt.z_value=z;
		pt2d = get3DTransform(pt);
		drawPixel(pt2d.x,pt2d.y,GREEN1);
	}
//BLUE RIGHT SIDE FILL
for(int z=60;z>=10;z--)
	for(int x=50;x>=0;x--)
	{
		Pts3D pt; Pts2D pt2d;
		pt.x_value=x; pt.y_value=50; pt.z_value=z;
		pt2d = get3DTransform(pt);
		drawPixel(pt2d.x,pt2d.y,BLUE1);
	}
// LIGHT RAYS DRAWLINES FRONT
drawLine(P.X[13],P.Y[13],P.X[11],P.Y[11],GREY1);
drawLine(P.X[14],P.Y[14],P.X[11],P.Y[11],GREY1);
drawLine(P.X[15],P.Y[15],P.X[11],P.Y[11],GREY1);

}


int main (void)
{

	 uint32_t pnum = PORT_NUM;
	 pnum = 1 ;
	 if ( pnum == 1 )
		 SSP1Init();
	 else
		 puts("INCORRECT PORT NUMBER");
	 lcd_init();
	 fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, BLACK);

	 drawCube();

	  return 0;

}
