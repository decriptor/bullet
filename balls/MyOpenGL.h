#ifndef _MYOPENGL_
#define _MYOPENGL_

#include <GL/gl.h>
#include <GL/glut.h>

class TheBalls;
int glutmain(int argc, char **argv, int width, int height, const char* title, TheBalls* balls);

#endif
