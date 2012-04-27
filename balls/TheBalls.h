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

	btScalar colors[12];

	int _width;
	int _height;
	
	GLfloat m_ShootBoxInitialSpeed;
	btBoxShape* m_shootBoxShape;
	btScalar m_defaultContactProcessingThreshold;
	bool m_cleanup;
	void DrawBalls(bool up);
	void DrawFloor();
	double m_floorSize;
	
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
		
		void shootBox(const btVector3& destination);
		void keyboardCallback(unsigned char key, int x, int y);
		void setShootBoxShape ();
		btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);

		static TheBalls* Create()
		{
			TheBalls* balls = new TheBalls;
			balls->myinit();
			balls->initPhysics();
			return balls;
		}

};

#endif
