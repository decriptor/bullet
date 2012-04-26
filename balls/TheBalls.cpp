#include <GL/gl.h>
#include <GL/glut.h>

#include "TheBalls.h"

TheBalls::TheBalls()
{
}

TheBalls::~TheBalls()
{
	exitPhysics();
}

void
TheBalls::reshape(int w, int h)
{
	_width = w;
	_height = h;
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(100.0, (GLfloat)w/(GLfloat)h, 0.5, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// glOrtho Left, Right, Bottom, Top, Near, Far
	glOrtho(-1.0, 1.5, -1.0, 1.0, -1.0, 1.0);
}

void
TheBalls::renderme()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt( 0.0, 0.0, 115.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);


	for (int j = m_dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			btTransform trans;
			body->getMotionState()->getWorldTransform(trans);
			glPushMatrix(); {
				GLfloat x = trans.getOrigin().getX();
				GLfloat y = trans.getOrigin().getY();
				GLfloat z = trans.getOrigin().getZ();

				glTranslatef(x, y, z);
				glColor3f(red.r, red.g, red.b);
				glutSolidSphere(2.0, 15, 15);
			} glPopMatrix();
		}
	}



	/*
	   if(m_dynamicsWorld)
	   {
	   glDisable(GL_CULL_FACE);
	   btScalar m[16];
	   btMatrix3x3 rot;
	   rot.setIdentity();
	   const int numObjects = m_dynamicsWorld->getNumCollisionObjects();
	   okAt( 0.0, 0.0, 115.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);


	   for(int i = 0; i < numObjects; i++)
	   {
	   btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
	   btRigidBody* body = btRigidBody::upcast(colObj);

	   if(body && body->getMotionState())
	   {
	   btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
	   myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
	   rot=myMotionState->m_graphicsWorldTrans.getBasis();
	   }
	   else
	   {
	   colObj->getWorldTransform().getOpenGLMatrix(m);
	   rot=colObj->getWorldTransform().getBasis();
	   }

	   btVector3 aabbMin,aabbMax;
	   m_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin,aabbMax);

	   aabbMin-=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	   aabbMax+=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);

	// m_shapeDrawer->drawOpenGL(m,colObj->getCollisionShape(),wireColor,getDebugMode(),aabbMin,aabbMax);
	}
	}
	*/
	glFlush();
	glutSwapBuffers();
}

void
TheBalls::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1.f/60.f, 10);
	}

	renderme();

	swapBuffers();
	glFlush();
}

void
TheBalls::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	swapBuffers();
	glFlush();
}

void
TheBalls::swapBuffers()
{
	glutSwapBuffers();
}

void
TheBalls::myinit()
{

	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*      light_position is NOT default value     */
	GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	
	glClearColor(btScalar(0.7),btScalar(0.7),btScalar(0.7),btScalar(0));
	
	//  glEnable(GL_CULL_FACE);
	//  glCullFace(GL_BACK);
}

void
TheBalls::initPhysics()
{
	//setTexturing(true);
	//setShadows(true);
	
	// collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	// use the default collision dispatcher.
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	//btDbvtBroadphase is a good general purpose broadphase.
	m_broadphase = new btDbvtBroadphase();

	// the default constraint solver.
	m_solver = new btSequentialImpulseConstraintSolver;

	// Create our dynamic world
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	// Set the gravity in our dynamic world
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));


	// Create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-56,0));

	{
		btScalar mass(0.);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if(isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		// add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	{
		btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		m_collisionShapes.push_back(colShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		startTransform.setOrigin(btVector3(2,10,0));

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody(body);
	}
}

void TheBalls::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void TheBalls::exitPhysics()
{
	// remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
			delete body->getMotionState();

		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	// delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	m_collisionShapes.clear();

	// delete dynamics world
	delete m_dynamicsWorld;

	// delete solver
	delete m_solver;

	// delete broadphase
	delete m_broadphase;

	// delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

