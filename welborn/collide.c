#include <GL/gl.h>
#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "btBulletDynamicsCommon.h"

/* Globals */

#define ON 1
#define OFF 0
#define FLOOR -1.0
#define CEILING 1.0
#define LEFT -1.0
#define RIGHT 1.0
#define FRONT -1.0
#define BACK 1.0
#define G 9.8 // meters/s^2
#define BALL_R 0.05



static double d2r = M_PI/180.0; /* degrees to radians */
static double r2d = 180.0/M_PI; /* radians to degrees */ 

GLfloat WallHeight = 0.2;
GLfloat WallThick  = 0.2;
GLint T = 0;
GLfloat iT = 0.04;

typedef struct {
  GLfloat r;
  GLfloat g;
  GLfloat b;
} rgb;


static rgb azure = {0.94, 0.93, 0.93};
static rgb coral = {1.0, 0.5, 0.3};
static rgb paleGreen = {0.6, 0.9, 0.6};

typedef struct {
  GLfloat x;
  GLfloat y;
  GLfloat z;
} point;

typedef struct {
  rgb   color;
  point loc;
  point vec;
  point v;
  GLfloat mass;
} Ball;

#define MAX_BALLS 1000
GLint ballCnt = 0;
Ball b[MAX_BALLS];

GLfloat lightPosition0[] = {-0.75, 1.0, 2.0, 1.0}; 
GLfloat lightAmbient0[] =  {0.0, 0.0, 0.0, 1.0}; // Default
GLfloat lightDiffuse0[] =  {1.0, 1.0, 1.0, 1.0}; // Default
GLfloat lightSpecular0[] = {1.0, 1.0, 1.0, 1.0}; // Default
GLfloat globalAmbientLight[] = {0.2, 0.2, 0.2, 1.0}; 

static int width = 1200, height = 800; // Init window size



void lighting(int flag) {
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular0);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, 
		   globalAmbientLight);       
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 
		  GL_TRUE); 

    // Enable Lighting and Hidden Surface Removal
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);  // Must be after window init & create
}

void init() {
  int i;

  glClearColor(azure.r, azure.g, azure.b, 1.0);
  glShadeModel(GL_SMOOTH);
  lighting(ON);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

}

void ibox() {
  WallHeight += ((T % 25) == 0) ? 0.1 : 0.0;

  glBegin(GL_QUADS); 
  {
    glColor3f(paleGreen.r, paleGreen.g, paleGreen.b);
    // front side
    glNormal3f(0.0, 0.0, -1.0);
    glVertex3f(LEFT,             FLOOR,              FRONT+0.01);
    glVertex3f(LEFT,             FLOOR + WallHeight, FRONT+0.01);
    glVertex3f(LEFT + WallThick, FLOOR + WallHeight, FRONT+0.01);
    glVertex3f(LEFT + WallThick, FLOOR,              FRONT+0.01);
  }  
  glEnd();
}

GLfloat imag(int j) {
  return sqrtf((b[j].vec.x * b[j].vec.x) +
	       (b[j].vec.y * b[j].vec.y) +
	       (b[j].vec.z * b[j].vec.z));
}

//  Dot product with vector along X-axis
// Only the x component matter as other values for X-axis are 0.0
GLfloat idotX(int j) {
  // dot with [1.0, 0.0, 0.0]
  return b[j].vec.x;
}

// Angle in radians between [1.0, 0.0, 0.0] and vector i
GLfloat iangle(int i, GLfloat mag) {
  return acos(idotX(i)/mag);
}


// Motion based on Physics Formulas
// from "Physics for Game Developers" pg 103
void computeMotion(int j) {
  GLfloat x, y, z;
  GLfloat m = imag(j);
  GLfloat a = iangle(j, m); // in radians

  printf("%d - (%f, %f, %f) [%f, %f, %f]\n", j, 
	 b[j].loc.x,
	 b[j].loc.y,
	 b[j].loc.z,
	 b[j].vec.x,
	 b[j].vec.y,
	 b[j].vec.z
);
	          
  // Location x, y, z @ time iT
  x = b[j].loc.x + 
              (b[j].vec.x * cos(a) * iT); // x(t)
  y = b[j].loc.y + 
              (b[j].vec.y * sin(a) * iT) - (G*iT*iT)/2.0; // y(t)
  z = b[j].vec.z; // z(t)

  // Compute a normalized vector in the direction of
  // the movement of the ball
  b[j].v.x = x - b[j].loc.x;
  b[j].v.y = y - b[j].loc.y;
  b[j].v.z = z - b[j].loc.z;

  m = sqrt(b[j].v.x*b[j].v.x + 
	   b[j].v.y*b[j].v.y +
	   b[j].v.z*b[j].v.z);

  // normalize
  b[j].v.x /= m;
  b[j].v.y /= m;
  b[j].v.z /= m;

  // Assign the new location
  b[j].loc.x = x;
  b[j].loc.y = y;
  b[j].loc.z = z;

  if (b[j].loc.x > 1.5) return; // too far left to care about


    // Check if on Floor
    if (b[j].loc.y <= FLOOR + BALL_R) {
      b[j].vec.y *= -1.0;
      b[j].loc.y = FLOOR + BALL_R;
    }
    else {
      b[j].vec.y = b[j].vec.y-(G*iT);
    }

}

#define PERFECTLY_ELASTIC 1.0   // Pool Balls
#define PERFECTLY_INELASTIC 0.0 // Wet Clay Balls

GLfloat E = PERFECTLY_ELASTIC;  // Cofficient Of Restitution

GLfloat collisionEquation(GLfloat m1, GLfloat v1, GLfloat m2, GLfloat v2) {
  return (((m1 - (E*m2))*v1) + ((1.0 + E)*m2*v2)) / (m1 + m2);
}

/*

*/
void collisionResponseV1(int j) {
  int i;
  GLfloat dx, dy, dz, d;
  GLfloat min = 2 * BALL_R;
  GLfloat fudge = BALL_R * 2.10; 
  GLfloat vj, vi, pjd, pid; // New Velocity
  GLfloat nx, ny, nz, nt;

  for (i = 0; i < ballCnt; i++) {
    if (i == j) continue; 
    if (b[i].loc.x > 1.5) continue; // Off the rightside

    // Compute distance between balls i & j
    dx = b[i].loc.x - b[j].loc.x;
    dy = b[i].loc.y - b[j].loc.y;
    dz = b[i].loc.z - b[j].loc.z;
    d = sqrtf(dx*dx + dy*dy + dz*dz);

    if (d <= fudge) {
      printf("V1 Collision between %d & %d\n", j, i); fflush(stdout);
      printf("%d [%f, %f, %f]\n", j, b[j].vec.x, b[j].vec.y, b[j].vec.z);
      printf("%d [%f, %f, %f]\n", i, b[i].vec.x, b[i].vec.y, b[i].vec.z);

      nx = b[j].loc.x - b[i].loc.x;
      ny = b[j].loc.y - b[i].loc.y;
      nz = b[j].loc.z - b[i].loc.z;
      
      // Normalized vector between balls
      nt = sqrtf(nx*nx + ny*ny + nz*nz);

      nx = nx / nt;
      ny = ny / nt;
      nz = nz / nt;

      printf("v=[%f, %f, %f]\n", nx, ny, nz); 

      // Projection Velocity
      GLfloat pj = 
	b[j].vec.x * nx +
	b[j].vec.y * ny +
	b[j].vec.z * nz;
	
      GLfloat pi =
	b[i].vec.x * nx +
	b[i].vec.y * ny +
	b[i].vec.z * nz;

      printf("vpj=%f, vnj=%f\n", pj, 0.0);
      printf("vpi=%f, vni=%f\n", pi, 0.0);

      
      // Projection Velocity after collision
      pjd = collisionEquation(b[j].mass, pj, b[i].mass, pi);
      pid = collisionEquation(b[i].mass, pi, b[j].mass, pj);

      printf("pcvj=%f\n", pjd);
      printf("pcvi=%f\n", pid);

      GLfloat x, y, z;
      /* Using V2 */
      x = b[j].vec.x + (pjd - pj)*nx;
      y = b[j].vec.y + (pjd - pj)*ny;
      z = b[j].vec.z + (pjd - pj)*nz;
      printf("%d' [%f, %f, %f]\n", j, x, y, z);

      x = b[i].vec.x + (pjd - pj)*nx;
      y = b[i].vec.y + (pjd - pj)*ny;
      z = b[i].vec.z + (pjd - pj)*nz;
      printf("%d' [%f, %f, %f]\n", i, x, y, z);
    }
  }
}

/*
  As defined in "Physics Modeling for Game Progamming", pg 190
  1) Find line-of-action (loa)
  2) Find velocity components along
     a) line-of-action (vp*)
     b) normal to line-of-action (vn*)
  3) Find post-collision velocity (pcv*)
  4) Rotate post-collision velocity 
 */
void collisionResponseV2(int j) {
  int i;
  GLfloat R2 = BALL_R * 2.0;
  GLfloat fudge = BALL_R * 2.01; 

  for (i = 0; i < ballCnt; i++) {
    if (i == j) continue; // same ball
    if (b[i].loc.x > 1.5) continue; // Off the rightside

    // Compute distance between balls i & j
    GLfloat dx, dy, dz, d;
    dx = b[i].loc.x - b[j].loc.x;
    dy = b[i].loc.y - b[j].loc.y;
    dz = b[i].loc.z - b[j].loc.z;
    d = sqrtf(dx*dx + dy*dy + dz*dz);

    if (d <= fudge) {
      printf("V2 Collision between %d & %d\n", j, i); fflush(stdout);
      printf("%d [%f, %f, %f]\n", j, b[j].vec.x, b[j].vec.y, b[j].vec.z);
      printf("%d [%f, %f, %f]\n", i, b[i].vec.x, b[i].vec.y, b[i].vec.z);

      // Correct for penetrating balls
      // Reverse the direction of the ball
      d = R2 - d;
      if (d > 0.0) {
	printf("Inside %d, %d\n", j, i);
	b[j].loc.x += -1 * b[j].v.x * d/2;
	b[j].loc.y += -1 * b[j].v.y * d/2;
	b[j].loc.z += -1 * b[j].v.z * d/2;

	b[i].loc.x += -1 * b[i].v.x * d/2;
	b[i].loc.y += -1 * b[i].v.y * d/2;
	b[i].loc.z += -1 * b[i].v.z * d/2;

      }

      // 1)  Find line-of-action (loa)
      // 1.1) Compute vector v from j & i
      point v;
      v.x = b[j].loc.x - b[i].loc.x;
      v.y = b[j].loc.y - b[i].loc.y;
      v.z = b[j].loc.z - b[i].loc.z;
      // 1.2) Normalize v
      GLfloat m = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
      v.x /= m;
      v.y /= m;
      v.z /= m;
      printf("v=[%f, %f, %f]\n", v.x, v.y, v.z); 


      // 1.3) line-of-action is the angle between
      //      v and [1.0, 0.0, 0.0]
      // Note: for dot product
      //       1.0*X = X
      //       0.0*X = 0.0
      GLfloat loa = acos(v.x); // in radians
      printf("loa = %f\n", loa); 
      
      // 2) Find velocity components along
      // 2.1) velocity component along line-of-action
      GLfloat vpj = b[j].vec.x * cos(loa) + b[j].vec.y * sin(loa);
      GLfloat vpi = b[i].vec.x * cos(loa) + b[i].vec.y * sin(loa);;
      
      // 2.2) velocity component along normal
      GLfloat vnj   = -b[j].vec.x * sin(loa) + b[j].vec.y * cos(loa);
      GLfloat vni   = -b[i].vec.x * sin(loa) + b[i].vec.y * cos(loa);

      printf("vpj=%f, vnj=%f\n", vpj, vnj);
      printf("vpi=%f, vni=%f\n", vpi, vni);

      // 3) Find post-collision velocity (pcv*)
      // 3.1) post-collision velocity
      GLfloat pcvj = collisionEquation(b[j].mass, vpj, b[i].mass, vpi); 
      GLfloat pcvi = collisionEquation(b[i].mass, vpi, b[j].mass, vpj); 

      printf("pcvj=%f\n", pcvj);
      printf("pcvi=%f\n", pcvi);

      // 4) Rotate post-collision velocity 
      b[j].vec.x = pcvj * cos(loa) - vnj * sin(loa);
      b[j].vec.y = pcvj * sin(loa) + vnj * cos(loa);

      b[i].vec.x = pcvi * cos(loa) - vni * sin(loa);
      b[i].vec.y = pcvi * sin(loa) + vni * cos(loa);

      printf("%d [%f, %f, %f]\n", j, b[j].vec.x, b[j].vec.y, b[j].vec.z);
      printf("%d [%f, %f, %f]\n", i, b[i].vec.x, b[i].vec.y, b[i].vec.z);

    }
  }
}

void imove(int k) {
   int i;
   T++;       // Counter

  for (i = 0; i < ballCnt; i++) {
    // Move Ball
    computeMotion(i);
    // collisionResponseV1(i);
    collisionResponseV2(i);
  }

  if ((ballCnt < MAX_BALLS) && ((T % 10) == 0)) {
    // Introduce next ball 
    // Random Color
    b[ballCnt].color.r = ((GLfloat)rand() / (GLfloat)RAND_MAX);
    b[ballCnt].color.g = ((GLfloat)rand() / (GLfloat)RAND_MAX);
    b[ballCnt].color.b = ((GLfloat)rand() / (GLfloat)RAND_MAX);

    // Always start on top of the wall
    b[ballCnt].loc.x = LEFT + BALL_R;
    b[ballCnt].loc.y = (FLOOR + WallHeight) + BALL_R;
    b[ballCnt].loc.z = 0.0;

    // x velocity has a random component
    b[ballCnt].vec.x = BALL_R * (15.0 + (rand() % 45));
    b[ballCnt].vec.y = 0.0;
    b[ballCnt].vec.z = 0.0;

    // mass is the same for all balls
    b[ballCnt].mass = 3.0;

    ballCnt++;
  }

  glutPostRedisplay();
}

void iball() {
  int i;

  for (i = 0; i < ballCnt; i++) {
    // Render balls
    glPushMatrix(); {
      glTranslatef(b[i].loc.x, b[i].loc.y, b[i].loc.z);
      glColor3f(b[i].color.r, b[i].color.g, b[i].color.b);
      glutSolidSphere(BALL_R, 40, 40);
    } glPopMatrix();
  }

}

void idisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_COLOR_MATERIAL);
  ibox();
  iball();
  glDisable(GL_COLOR_MATERIAL);
  glutTimerFunc(16, imove, 0);
  glutSwapBuffers();
}




void keyboard(unsigned char key, int x, int y) {
  switch(key) {
  case 'q':
    exit(0);
    break;
    
  case 'E':                                 // Changing Cofficient Of Restitution
    if (E < PERFECTLY_ELASTIC) E += 0.05;
    break;
  case 'e':
    if (E > PERFECTLY_INELASTIC) E -= 0.05;
    break;
  }
}

void reshape(int w, int h) {
  width = w;
  height = h;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // glOrtho Left, Right, Bottom, Top, Near, Far
  glOrtho(-1.0, 1.5, -1.0, 1.0, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glViewport(0, 0, w, h);
}

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;
btDiscreteDynamicsWorld             * dynamicsWorld = NULL;
btSequentialImpulseConstraintSolver * solver = NULL;
btBroadphaseInterface               * overlappingPairCache = NULL;
btCollisionDispatcher               * dispatcher = NULL;
btDefaultCollisionConfiguration     * collisionConfiguration = NULL;
void finishBullet() {
	//delete collision shapes
	for (int j=0;j<collisionShapes.size();j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();

}

// Bullet Objects
btCollisionShape                    * groundShape = NULL;


void initBullet()
{

	int i;

	///collision configuration contains default setup for memory, collision setup. 
	// Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can 
	// use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. 
	// You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different 
	// solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));


	collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-56,0));

	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}


	{
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

			startTransform.setOrigin(btVector3(2,10,0));
		
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			dynamicsWorld->addRigidBody(body);
	}



/// Do some simulation



	for (i=0;i<100;i++)
	{
		dynamicsWorld->stepSimulation(1.f/60.f,10);
		
		//print positions of all objects
		for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				btTransform trans;
				body->getMotionState()->getWorldTransform(trans);
				printf("world pos = %f,%f,%f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
			}
		}
	}


	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	finishBullet();
}


int main(int argc, char **argv) {

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(width, height);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Balls");

  init();

  glutDisplayFunc(idisplay);
  glutKeyboardFunc(keyboard);
  glutReshapeFunc(reshape);
  glutMainLoop();
}
    
    
    
