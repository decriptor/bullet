#ifndef _COLORS_
#define _COLORS_
// Colors
#include <GL/gl.h>

// Color Structure
typedef struct {
	GLfloat r;     // Red
	GLfloat g;     // Green
	GLfloat b;     // Blue
} rgb;

// Colors
static rgb black   = {0.0, 0.0, 0.0};
static rgb red     = {1.0, 0.0, 0.0};
static rgb green   = {0.0, 1.0, 0.0};
static rgb yellow  = {1.0, 1.0, 0.0};
static rgb blue    = {0.0, 0.0, 1.0};
static rgb gray    = {0.5, 0.5, 0.5};
static rgb magenta = {1.0, 0.0, 1.0};
static rgb cyan    = {0.0, 1.0, 1.0};
static rgb white   = {1.0, 1.0, 1.0};
static rgb orange  = {1.0, 0.6, 0.0};
static rgb brown   = {165.0/255.0, 42.0/255.0, 42.0/255.0};

// Colors from sun.c
static rgb violet = {238.0/255.0, 130.0/255.0, 238.0/255.0};
static rgb olive = {192.0/255.0, 1.0, 62.0/255.0};
static rgb khaki = {1.0, 246.0/255.0, 143.0/255.0};
static rgb gold = {1.0, 215.0/255.0, 0.0};
static rgb paleGreen = {0.6, 0.9, 0.6};
static rgb deepSkyBlue = {0.0, 0.74, 1.0};
static rgb lightSkyBlue = {100.0/255.0, 226.0/255.0, 1.0};
static rgb offWhite = {0.87, 0.87, 0.87};
static rgb antiqueWhite = {1.0, 0.93, 0.83};
static rgb peachPuff = {0.93, 0.81, 0.678};
static rgb azure = {0.94, 0.93, 0.93};
static rgb coral = {1.0, 0.5, 0.3};
static rgb purple = {0.62, 0.125, 0.94};
static rgb aqua = {127.0/255.0, 1.0, 212.0/255.0};
static rgb seaGreen = {0.329, 1.0, 0.6};
static rgb lavenderBlush = {1.0, 240.0/255.0, 245.0/255.0};
static rgb honeydew = {0.941, 1.0, 0.941};

#endif
