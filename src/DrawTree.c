/*
===============================================================================
 Name        : 2019F_LAB1_014525420.c
 Author      : AKHIL CHERUKURI
 Version     : FINAL
 Copyright   : $(copyright)
 Description : Lab LPC-ARM Architecture Based 2D and 3D Graphics Processing Engine Design
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

// Self Defined GREEN Shades
#define GREEN1 0x38761D
#define GREEN2 0x002200
#define GREEN3 0x00FC7C
#define GREEN4 0x32CD32
#define GREEN5 0x228B22
#define GREEN6 0x006400


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
 printf("Lab LPC-ARM Architecture Based 2D and 3D Graphics Processing Engine Design\n");
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

// Coordinate to Cartesian
int16_t xCartesian(int16_t x){
	x = x + (_width>>1);
	return x;
}

int16_t yCartesian(int16_t y){
	y = (_height>>1) - y;
	return y;
}

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



//2D Engine

#define DeltaX  0
#define DeltaY 	0
#define pi 3.1416
// given lamda = 0.8
float lambda = 0.8;

// random lamda values
int rand_lambda = 5;
float array_lamda[] = {0.8-0.1, 0.8-0.05, 0.8, 0.8+0.05, 0.8+0.1};
int angles = 8;

// alpha degrees =5,10,15,20,30,40,50,60
float alpha[] = {pi/36, pi/18, pi/12, pi/9, pi/6, (2*pi)/9, (5*pi)/18, pi/3};

// colors for green leaves
uint32_t leaves[7] = {GREEN1, GREEN2, GREEN3, GREEN , GREEN4, GREEN5, GREEN6};


struct point{
	int x;
	int y;
};

//
struct point temp_point(int x, int y){
	struct point temp;
	temp.x = x;
	temp.y = y;

	return temp;
}

// Pre-processing

// Translate X Coordinate
// Tree branch reduction
// x= x1 + 0.8 * (x1-x0)

int16_t new_x(int16_t x0, int16_t x1){
	int16_t x;
	x = x1 + lambda*(x1 - x0);
	return x;
}

// Translate Y Coordinate
// Tree branch reduction
// y= y1 + 0.8 * (y1-y0)

int16_t new_y(int16_t y0, int16_t y1){
	int16_t y;
	y = y1 + lambda*(y1 - y0);
	return y;
}

struct point new_xy(struct point p0, struct point p1){
	struct point point;
	float z = array_lamda[rand()%rand_lambda];

	point = temp_point(p1.x + z * (p1.x - p0.x), p1.y + z * (p1.y - p0.y));

	return point;
}

// Random XY

struct point rand_xy(void){
return temp_point(rand()%64 - 32, rand()%80 - 60);
}

// CLOCKWISE BRANCH

struct point CW(struct point p0, struct point p1){
	struct point delta, point, prime, doubleprime, temp;
	// random angles '-'
	float deg = -alpha[rand()%angles];

	point = new_xy(p0, p1);
	delta = temp_point(-p1.x, -p1.y);
	prime = temp_point(point.x + delta.x, point.y + delta.y);
	doubleprime = temp_point(cos(deg)*prime.x - sin(deg)*prime.y, sin(deg)*prime.x + cos(deg)*prime.y);
	temp = temp_point(doubleprime.x - delta.x, doubleprime.y - delta.y);

	return temp;
}

// COUNTER CLOCKWISE BRANCH

struct point CCW(struct point p0, struct point p1){
    struct point delta, point, prime, doubleprime, temp;
    // random angles '+'
    float deg = alpha[rand()%angles];

    point = new_xy(p0, p1);
	delta = temp_point(-p1.x, -p1.y);
	prime = temp_point(point.x + delta.x, point.y + delta.y);
	doubleprime = temp_point(cos(deg)*prime.x - sin(deg)*prime.y, sin(deg)*prime.x + cos(deg)*prime.y);
	temp = temp_point(doubleprime.x - delta.x, doubleprime.y - delta.y);

	return temp;
}

// MIDDLE BRANCH

struct point NW(struct point p0, struct point p1){
	struct point point_nw;

	point_nw = temp_point(new_x(p0.x, p1.x), new_y(p0.y, p1.y));

	return point_nw;
}


void drawTree(){
// given least lvls=10
int lvls = 11;
struct point b[lvls + 2];
int z;
int ccw = 0, nw = 1, cw = 2, s = 3;
int al[lvls];

while(1){
struct point root = rand_xy();
z = rand()%4;

b[0] = temp_point(root.x, root.y);
b[1] = temp_point(root.x, root.y + 20);
drawLine(b[0].x,b[0].y,b[1].x,b[1].y, leaves[z]);

for(al[0] = 0; al[0] < s; al[0]++){
	if(al[0] == ccw){
		b[2] = CCW(b[0], b[1]);
				}
	else if(al[0] == nw){
		b[2] = NW(b[0], b[1]);
				}
	else if(al[0] == cw){
		b[2] = CW(b[0], b[1]);
				}
drawLine(b[1].x, b[1].y, b[2].x, b[2].y, leaves[z]);

for(al[1] = 0; al[1] < s; al[1]++){
	if(al[1] == ccw){
		b[3] = CCW(b[1], b[2]);
					}
	else if(al[1] == nw){
		b[3] = NW(b[1], b[2]);
					}
	else if(al[1] == cw){
		b[3] = CW(b[1], b[2]);
					}
drawLine(b[2].x, b[2].y, b[3].x, b[3].y, leaves[z]);

for(al[2] = 0; al[2] < s; al[2]++){
	if(al[2] == ccw){
		b[4] = CCW(b[2], b[3]);
						}
	else if(al[2] == nw){
		b[4] = NW(b[2], b[3]);
						}
	else if(al[2] == cw){
		b[4] = CW(b[2], b[3]);
						}
drawLine(b[3].x, b[3].y, b[4].x, b[4].y, leaves[z]);

for(al[3] = 0; al[3] < s; al[3]++){
	if(al[3] == ccw){
		b[5] = CCW(b[3], b[4]);
							}
	else if(al[3] == nw){
		b[5] = NW(b[3], b[4]);
							}
	else if(al[3] == cw){
		b[5] = CW(b[3], b[4]);
							}

drawLine(b[4].x, b[4].y, b[5].x, b[5].y, leaves[z]);

for(al[4] = 0; al[4] < s; al[4]++){
	if(al[4] == ccw){
		b[6] = CCW(b[4], b[5]);
								}
	else if(al[4] == nw){
		b[6] = NW(b[4], b[5]);
								}
	else if(al[4] == cw){
		b[6] = CW(b[4], b[5]);
								}
drawLine(b[5].x, b[5].y, b[6].x, b[6].y, leaves[z]);

for(al[5] = 0; al[5] < s; al[5]++){
	if(al[5] == ccw){
		b[7] = CCW(b[5], b[6]);
									}
	else if(al[5] == nw){
		b[7] = NW(b[5], b[6]);
									}
	else if(al[5] == cw){
		b[7] = CW(b[5], b[6]);
									}
drawLine(b[6].x, b[6].y, b[7].x, b[7].y, leaves[z]);

for(al[6] = 0; al[6] < s; al[6]++){
	if(al[6] == ccw){
		b[8] = CCW(b[6], b[7]);
										}
	else if(al[6] == nw){
		b[8] = NW(b[6], b[7]);
									}
	else if(al[6] == cw){
		b[8] = CW(b[6], b[7]);
										}
drawLine(b[7].x, b[7].y, b[8].x, b[8].y, leaves[z]);

for(al[7] = 0; al[7] < s; al[7]++){
	if(al[7] == ccw){
		b[9] = CCW(b[7], b[8]);
											}
	else if(al[7] == nw){
		b[9] = NW(b[7], b[8]);
											}
	else if(al[7] == cw){
		b[9] = CW(b[7], b[8]);
											}
drawLine(b[8].x, b[8].y, b[9].x, b[9].y, leaves[z]);

for(al[8] = 0; al[8] < s; al[8]++){
	if(al[8] == ccw){
		b[10] = CCW(b[8], b[9]);
												}
	else if(al[8] == nw){
		b[10] = NW(b[8], b[9]);
												}
	else if(al[8] == cw){
		b[10] = CW(b[8], b[9]);
												}
drawLine(b[9].x, b[9].y, b[10].x, b[10].y, leaves[z]);

for(al[9] = 0; al[9] < s; al[9]++){
	if(al[9] == ccw){
		b[11] = CCW(b[9], b[10]);
													}
	else if(al[9] == nw){
		b[11] = NW(b[9], b[10]);
													}
	else if(al[9] == cw){
		b[11] = CW(b[9], b[10]);
													}
drawLine(b[10].x, b[10].y, b[11].x, b[11].y, leaves[z]);

									}
								}
							}
						}
					}
				}
			}
		}
	}
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
	float_t x_value;
	float_t y_value;
	float_t z_value;
}Pts3D;

Pts3D ShadowPoint3D(Pts3D Pi, Pts3D Ps, float lambda)
{
	Pts3D pt;
	pt.x_value = (Pi.x_value + (lambda*(Ps.x_value-Pi.x_value)));
	pt.y_value = (Pi.y_value + (lambda*(Ps.y_value-Pi.y_value)));
	pt.z_value = (Pi.z_value + (lambda*(Ps.z_value-Pi.z_value)));

	return pt;
}

void drawShadow(){
#define UpperBD 52
#define NumOfPts 10
#define Pi 3.1415926
typedef struct{
		float X[UpperBD];
		float Y[UpperBD];
		float Z[UpperBD];

	}pworld;
typedef struct{
		float X[UpperBD];
		float Y[UpperBD];
		float Z[UpperBD];

	}pviewer;
typedef struct{
		float X[UpperBD];
		float Y[UpperBD];
	}pperspective;

// E(200,200,200),

float_t Xe=200;
float_t Ye=200;
float_t Ze=200;

float_t Rho=sqrt(pow(Xe,2)+pow(Ye,2)+pow(Xe,2));
float_t D_focal=40;
float_t L1,L2,L3,L4;

pworld WCS;
pviewer V;
pperspective P;

// X Y Z World Coordinate System

WCS.X[0]=0.0; WCS.Y[0]=0.0; WCS.Z[0]=0.0; //origin
WCS.X[1]=50.0; WCS.Y[1]=0.0; WCS.Z[1]=0.0;
WCS.X[2]=0.0; WCS.Y[2]=50.0; WCS.Z[2]=0.0;
WCS.X[3]=0.0; WCS.Y[3]=0.0;	WCS.Z[3]=50.0;

// Elevate Cube along Zw axis by 10

WCS.X[4]=100.0; WCS.Y[4]=0; WCS.Z[4]=10.0;
WCS.X[5]=0.0; WCS.Y[5]=100.0; WCS.Z[5]=10.0;
WCS.X[6]=0.0; WCS.Y[6]=0.0; WCS.Z[6]=110.0;

WCS.X[7]=100.0; WCS.Y[7]=0.0; WCS.Z[7]=110.0;
WCS.X[8]=100.0; WCS.Y[8]=100.0; WCS.Z[8]=10.0;
WCS.X[9]=0.0; WCS.Y[9]=100.0; WCS.Z[9]=110.0;

WCS.X[10]=100.0; WCS.Y[10]=100.0; WCS.Z[10]=110.0;

// POINT OF LIGHT SOURCE PS(-50, 50, 300)

WCS.X[11]=-50.0; WCS.Y[11]=50.0; WCS.Z[11]=300.0;


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
	V.X[i]=-sPheta*WCS.X[i]+cPheta*WCS.Y[i];
	V.Y[i] = -cPheta * cPhi * WCS.X[i]- cPhi * sPheta * WCS.Y[i]+ sPhi * WCS.Z[i];
	V.Z[i] = -sPhi * cPheta * WCS.X[i]- sPhi * cPheta * WCS.Y[i]-cPheta * WCS.Z[i] + Rho;
}

for(int i=0;i<=UpperBD;i++)
{
	P.X[i]=D_focal*V.X[i]/V.Z[i];
	P.Y[i]=D_focal*V.Y[i]/V.Z[i];

}

// CUBE DRAWLINES

drawLine(P.X[7],P.Y[7],P.X[4],P.Y[4],RED);
drawLine(P.X[7],P.Y[7],P.X[6],P.Y[6],RED);
drawLine(P.X[7],P.Y[7],P.X[10],P.Y[10],RED);
drawLine(P.X[8],P.Y[8],P.X[4],P.Y[4],RED);
drawLine(P.X[8],P.Y[8],P.X[5],P.Y[5],RED);
drawLine(P.X[8],P.Y[8],P.X[10],P.Y[10],RED);
drawLine(P.X[9],P.Y[9],P.X[6],P.Y[6],RED);
drawLine(P.X[9],P.Y[9],P.X[5],P.Y[5],RED);
drawLine(P.X[9],P.Y[9],P.X[10],P.Y[10],RED);

// SHADOW DRAWLINES

drawLine(P.X[12],P.Y[12],P.X[13],P.Y[13],WHITE);
drawLine(P.X[13],P.Y[13],P.X[15],P.Y[15],WHITE);
drawLine(P.X[14],P.Y[14],P.X[15],P.Y[15],WHITE);
drawLine(P.X[14],P.Y[14],P.X[12],P.Y[12],WHITE);

// LIGHT RAYS DRAWLINES

drawLine(P.X[12],P.Y[12],P.X[11],P.Y[11],YELLOW);
drawLine(P.X[13],P.Y[13],P.X[11],P.Y[11],YELLOW);
drawLine(P.X[14],P.Y[14],P.X[11],P.Y[11],YELLOW);
drawLine(P.X[15],P.Y[15],P.X[11],P.Y[11],YELLOW);

// DISPLAY CONSOLE RAY EQUATION
//for(int i=0;i<=10;i++)
//printf("X[%d]=%f,Y[%d]=%f\n",i,perspective.X[i],i,perspective.Y[i]);
//printf("1:%f,2:%f,3:%f,4:%f \n",lambda1,lambda2,lambda3,lambda4);

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

	 fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, BLACK);
	 int choice;
	 printf("Enter 1 for 2D Tree and 2 for 3D Shadow");
	 scanf("%d",&choice);
	 if(choice == 1)
	 {
	 drawTree();
	 }
	 else if(choice == 2)
	 {
	 drawShadow();
	 }
	 else{
		 printf("Wrong Choice");
	 }


	  return 0;

}

