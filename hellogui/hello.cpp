#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
 
#include <btBulletDynamicsCommon.h>
 
#include "colors.h"

GLfloat x = 0.0;
GLfloat y = 0.0;
GLfloat z = 0.0;

static int windowX = 350;
static int windowY = 350;

void setup_opengl(int, char**);
void display();

int main (int argc, char** argv)
{
	// Setup openGL stuff
	setup_opengl(argc, argv);
	// Setup
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0,-10,0));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
	btCollisionShape* fallShape = new btSphereShape(1);
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);
	btDefaultMotionState* fallMotionState =
		new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)));
	btScalar mass = 1;
	btVector3 fallInertia(0,0,0);
	fallShape->calculateLocalInertia(mass,fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);

	dynamicsWorld->addRigidBody(fallRigidBody);


	// Calculate ball location
	for (int i=0 ; i<300 ; i++) {
		dynamicsWorld->stepSimulation(1/60.f,10);

		btTransform trans;
		fallRigidBody->getMotionState()->getWorldTransform(trans);
		x = trans.getOrigin().getX();
		y = trans.getOrigin().getY();
		z = trans.getOrigin().getZ();

		std::cout << "sphere x: " << x << " ";
		std::cout << "y: " << y << " ";
		std::cout << "z: " << z << std::endl;
		display();
	}

	// Clean up
	dynamicsWorld->removeRigidBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;
	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;
	delete fallShape;
	delete groundShape;
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	return 0;
}

//
// OPENGL stuff
//

void display() {
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	glPushMatrix();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glTranslatef(x, y, z);
	glutSolidSphere(.10, 15, 15);
	glPopMatrix();

	glutSwapBuffers();
	glFlush();
}

void init() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0, 1.0, -1.0, 1.0, -2.0, 2.0);        // Orthographic Perspective
	glClearColor(cyan.r, cyan.g, cyan.b, 0.0);       // Background Color
	glShadeModel(GL_SMOOTH);                         // Smooth Shading Model
}


void reshape(int w, int h) {
	windowX = w;
	windowY = h;
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0, 1.0, -1.0, 1.0, -2.0, 2.0);        // Orthographic Perspective
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void setup_opengl(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(windowX, windowY);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Bullet: Hello World w/ GUI");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMainLoop();
}
