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
TheBalls::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1.f/60.f, 10);
	}

	renderme();

	glFlush();
	swapBuffers();
}

void
TheBalls::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();
	glFlush();
	swapBuffers();
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
	btCollisionShape* groundShape = new btBoxShape(btVector(btScalar(50.),btScalar(50.),btScalar(50.)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector(0,-56,0));

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
		btCollisionShape* colShape = new btBoxShape(btVector(1,1,1));
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

