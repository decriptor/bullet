#include <math.h>

#ifndef _GLOBALS_
#define _GLOBALS_

// ascii code for the escape key
#define ESCAPE 27

// Stuff for PI
const GLfloat d2r = M_PI/180;
const GLfloat r2d = 180/M_PI;

// An array for holding textures
GLuint textures[2];
GLuint texture0 = 0;
GLuint texture1 = 1;

// The different textures
char *texture0_filename = "";

typedef struct {
	GLfloat x;
	GLfloat y;
	GLfloat z;
} point;

#endif
