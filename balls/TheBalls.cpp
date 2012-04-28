#include <GL/gl.h>
#include <GL/glut.h>

#include "TheBalls.h"

TheBalls::TheBalls()
{
	m_ShootBoxInitialSpeed = 10.f;
	m_floorSize = 400.;
	colors[0] = 0.8;
	colors[1] = 0.2;
	colors[2] = 0.2;
	colors[3] = 0.2;
	colors[4] = 0.8;
	colors[5] = 0.2;
	colors[6] = 0.2;
	colors[7] = 0.2;
	colors[8] = 0.8;
	colors[9] = 0.8;
	colors[10] = 0.8;
	colors[11] = 0.8;
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
	gluPerspective(80.0, (GLfloat)w/(GLfloat)h, 0.5, 2000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// glOrtho Left, Right, Bottom, Top, Near, Far
	glOrtho(-1.0, 1.5, -1.0, 1.0, -1.0, 1.0);
}

btRigidBody*
TheBalls::localCreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
        btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
                shape->calculateLocalInertia(mass,localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

        btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

        btRigidBody* body = new btRigidBody(cInfo);
        body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);


        m_dynamicsWorld->addRigidBody(body);

        return body;
}


void
TheBalls::renderme()
{
	glClearStencil(0);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt( 0.0, 130.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glDepthMask(GL_FALSE);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFF);
	glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
	{
		DrawFloor();
	}
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glDepthMask(GL_TRUE);
	glStencilFunc(GL_EQUAL, 1, 0xFFFFFFFF);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	
	
	glDisable(GL_DEPTH_TEST);
	glPushMatrix();{
		glScalef(1.0f, -1.0f, 1.0f);
		glTranslatef(0.0, -1.0, 0.0);
		{
			DrawBalls(false);
		}
	}glPopMatrix();
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	{
		DrawFloor();
	}
	glDisable(GL_BLEND);
	{
		DrawBalls(true);
	}
	glFlush();
	glutSwapBuffers();
}

void
TheBalls::DrawFloor()
{
	static btScalar matrix[16];
	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[0];
	btRigidBody* body = btRigidBody::upcast(obj);
	if (body && body->getMotionState())
	{
		btTransform trans;
		body->getMotionState()->getWorldTransform(trans);
		glPushMatrix(); {
			trans.getOpenGLMatrix(matrix);
			glMultMatrixf(matrix);
			glBegin(GL_QUADS);
			{
				glColor4f(.9, .9, .9, 0.7);
				glVertex3f(-m_floorSize, 1., -m_floorSize);
				glVertex3f(-m_floorSize, 1., m_floorSize);
				glVertex3f(m_floorSize, 1., m_floorSize);
				glVertex3f(m_floorSize, 1., -m_floorSize);
			}
			glEnd();
		} glPopMatrix();
	}
}

void
TheBalls::DrawBalls(bool up)
{
	static btScalar matrix[16];
	for (int j = up ? 1 : (m_dynamicsWorld->getNumCollisionObjects() - 1) ; up ?( j < m_dynamicsWorld->getNumCollisionObjects()) : (j >= 1); j+= up ? 1 : -1)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			
			btTransform trans;
			body->getMotionState()->getWorldTransform(trans);
			btScalar x = trans.getOrigin().getX();
			btScalar z = trans.getOrigin().getZ();
			if(up || (abs(x) <= m_floorSize + 1 && abs(z) <= m_floorSize + 1))
			{
				glColor3f(colors[j % 4 * 3], colors[j % 4 * 3 + 1], colors[j % 4 * 3 + 2]);
				
				glPushMatrix(); {
				trans.getOpenGLMatrix(matrix);
				glMultMatrixf(matrix);
					glutSolidSphere(10.0, 12, 12);
				} glPopMatrix();
			}
			else if(m_cleanup)
			{
				if (body && body->getMotionState())
					delete body->getMotionState();

				m_dynamicsWorld->removeCollisionObject(obj);
				delete obj;
			}
		}
	}
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

//	swapBuffers();
//	glFlush();
}

void
TheBalls::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

//	swapBuffers();
//	glFlush();
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
	glEnable(GL_COLOR_MATERIAL);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL);

	glDepthFunc(GL_LEQUAL);
	
	glClearColor(btScalar(0.4),btScalar(0.5),btScalar(0.8),btScalar(0));
	
	//  glEnable(GL_CULL_FACE);
	//  glCullFace(GL_BACK);
}

void
TheBalls::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q':
		exit(0);
		break;
	case '.':
		shootBox(btVector3(btScalar(0.f),btScalar(0.f),btScalar(-100.f)));
		break;
	case '=':
	case '+':
		m_ShootBoxInitialSpeed += 10.f;
		break;
	case '-':
		m_ShootBoxInitialSpeed -= 10.f;
		break;
	case 'r':
		clientResetScene();
		break;
	default:
		break;
	}
}

void
TheBalls::setShootBoxShape ()
{
        if (!m_shootBoxShape)
        {
                btBoxShape* box = new btBoxShape(btVector3(10.f,10.f,10.f));
                box->initializePolyhedralFeatures();
                m_shootBoxShape = box;
        }
}

void
TheBalls::shootBox(const btVector3& destination)
{
	if (m_dynamicsWorld)
        {
                float mass = 1.f;
                btTransform startTransform;
                startTransform.setIdentity();
                btVector3 camPos = btVector3(btScalar(0.f),btScalar(100.f),btScalar(200.f));
                startTransform.setOrigin(camPos);

                setShootBoxShape ();

                btRigidBody* body = this->localCreateRigidBody(mass, startTransform, m_shootBoxShape);
                body->setLinearFactor(btVector3(1,1,1));
                //body->setRestitution(1);

//                btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
                btVector3 linVel(0,30,-100);
                linVel.normalize();
                linVel*=m_ShootBoxInitialSpeed;
		//printf("the shoot speed %f\n", m_ShootBoxInitialSpeed);
                body->getWorldTransform().setOrigin(camPos);
                body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
                body->setLinearVelocity(linVel);
                body->setAngularVelocity(btVector3(0,0,0));
                body->setCcdMotionThreshold(0.5);
                body->setCcdSweptSphereRadius(0.9f);
        }
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
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(m_floorSize),btScalar(2.),btScalar(m_floorSize)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));

	{
		btScalar mass(0.0f);

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
	for(int i = 0; i < 400; i++)
	{
		btCollisionShape* colShape = new btSphereShape(btScalar(10.));
		m_collisionShapes.push_back(colShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(0.8f);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(rand() % 20 - 10,rand() % 20 - 10,rand() % 20 - 10);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		startTransform.setOrigin(btVector3(rand() % 200 - 100,100 + i * 20,2 + rand() % 200 - 100));

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

