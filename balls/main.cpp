#include "TheBalls.h"
#include "MyOpenGL.h"
#include "btBulletDynamicsCommon.h"

int main(int argc,char** argv)
{

	TheBalls balls;
	balls.initPhysics();

#ifdef CHECK_MEMORY_LEAKS
	balls.exitPhysics();
#else
	return glutmain(argc, argv, 1024, 600, "Bullet Final Project", &balls);
#endif
	
	//default glut doesn't return from mainloop
	return 0;
}

