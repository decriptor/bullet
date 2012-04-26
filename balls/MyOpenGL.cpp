#include "MyOpenGL.h"
#include "TheBalls.h"

static TheBalls* _theballs = 0;

static void glutMoveAndDisplayCallback()
{
	_theballs->clientMoveAndDisplay();
}

static void glutReshapeCallback(int w, int h)
{
	_theballs->reshape(w,h);
}

static void glutDisplayCallback(void)
{
	_theballs->displayCallback();
}

int glutmain(int argc, char **argv, int width, int height, const char* title, TheBalls* balls)
{
	_theballs = balls;

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);

	//_theballs->myinit();

	glutReshapeFunc(glutReshapeCallback);
	glutIdleFunc(glutMoveAndDisplayCallback);
	glutDisplayFunc(glutDisplayCallback);

	glutMoveAndDisplayCallback();

	glutMainLoop();
	return 0;
}
