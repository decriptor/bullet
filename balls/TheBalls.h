#ifndef _THEBALLS_
#define _THEBALLS_

#include "colors.h"
#include "btBulletDynamicsCommon.h"

class TheBalls
{
	// this is the most important class
	btDynamicsWorld* m_dynamicsWorld;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btSequentialImpulseConstraintSolver* m_solver;

	int _width;
	int _height;
	
	public:

		TheBalls();
		~TheBalls();

		void initPhysics();
		void exitPhysics();

		void myinit();

		btDynamicsWorld* getDynamicsWorld()
		{
			return m_dynamicsWorld;
		}

		void clientMoveAndDisplay();
		void displayCallback();
		void clientResetScene();

		void renderme();
		void reshape(int w, int h);
		void swapBuffers();
		void updateModifierKeys();

		static TheBalls* Create()
		{
			TheBalls* balls = new TheBalls;
			balls->myinit();
			balls->initPhysics();
			return balls;
		}

};

#endif
