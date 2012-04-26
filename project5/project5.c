#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <GL/gl.h>
#include <GL/glut.h>

#include "colors.h"
#include "texture.h"
#include "globals.h"


/* The number of our GLUT window */
int window;

// My textured sphere
GLint list[1];
GLUquadricObj *sphere;

// Lighting 0
GLfloat light_position[] = { -1.0, 1.0, 1.0, 0.0};
GLfloat white_light[] = { 1.0, 1.0, 1.0, 0.0};
GLfloat lmodel_ambient[] = { 1.0, 1.0, 1.0, 0.0};

int boid_count = 15;
Boid boids[15];

float gen_rand (float low, float high)
{
	int ilow = (int)(low * 100);
	int ihigh = (int)(high * 100);
	int r = rand() % ihigh;
	r += ilow;
	return (float)r/100;
}

void generateBoids() {
	int i;
	for(i = 0; i < boid_count; i++)
	{
		boids[i].position.x = gen_rand(-20.0, 50.0);
		boids[i].position.y = gen_rand(-30.0, 50.0);
		boids[i].position.z = gen_rand(-25.0, 40.0);

		boids[i].speed.x = 0.2;
		boids[i].speed.y = 0.2;
		boids[i].speed.z = 0.2;

		boids[i].max_speed = 1.0;
		boids[i].max_acceleration = 0.3;
	}
}

void generateObstacles()
{

}

void init () {
	glClearColor (black.r, black.g, black.b, 0.5);
	glMatrixMode(GL_PROJECTION);
	glShadeModel(GL_SMOOTH);
	glLoadIdentity ();

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lmodel_ambient);

	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);

	generateBoids();
	generateObstacles();

	texture0 = LoadTextureRAW (texture0_filename, 0, 1024, 1024);
	texture1 = LoadTextureRAW (texture1_filename, 0, 512, 512);

	// Setup my sphere
//	glEnable ( GL_CULL_FACE );
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glNewList(list[0], GL_COMPILE);
	sphere = gluNewQuadric();
	gluQuadricDrawStyle( sphere, GLU_FILL);
	gluQuadricNormals( sphere, GLU_SMOOTH);
	gluQuadricOrientation( sphere, GLU_OUTSIDE);
	gluQuadricTexture( sphere, GL_TRUE);
	gluSphere( sphere, 10, 16, 16);
	glEndList();
}

int attract = 0;
void move (int k) {
	
	if (attract < 50)
		attract++;
	else
		boids_B = 4000;
	// Create some random event that makes the boids 
	// not attracted to each other anymore (change A B)

	// Calculate the accel vector for each boid based on the different
	// forces around it and update its speed.
	int b;
	Vector accel;
	for (b = 0; b < boid_count; b++)
	{
		accel = jedi_force_walls(boids[b].position);
		accel = jedi_vector_add(accel, jedi_force_obstacles(boids[b].position));
		accel = jedi_vector_add(accel, jedi_force_boids(boids[b].position, boids, boid_count));


		// Apply acceleration to speed
		accel = jedi_speed(accel, origin, boids[b].max_acceleration);
//		boids[b].speed = jedi_vector_add(boids[b].speed, vrs);
		boids[b].speed = jedi_speed(boids[b].speed, accel, boids[b].max_speed);
	}

	// Now that speed has been updated, lets update the boid's position
	for (b = 0; b < boid_count; b++)
	{
		boids[b].position = jedi_vector_add(boids[b].position, boids[b].speed);
	}

	glutPostRedisplay();
}

int angle = 5;
void display ()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity ();
	gluLookAt( 0.0, 0.0, 115.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
//	glRotatef(angle += 1, 0.0, 1.0, 0.0);
	// Draw background
	glEnable(GL_TEXTURE_2D);
	glPushMatrix(); {
		glBindTexture(GL_TEXTURE_2D, texture0);
	//	glTranslatef(0.0, 0.0,-1.0);
	//	glRotatef(90, 1.0, 0.0, 0.0);
		glBegin(GL_POLYGON); {
			glTexCoord2f(1,0); glVertex3f( 350.0, 250.0,-70.0);
			glTexCoord2f(0,0); glVertex3f(-350.0, 250.0,-70.0);
			glTexCoord2f(0,1); glVertex3f(-350.0,-250.0,-70.0);
			glTexCoord2f(1,1); glVertex3f( 350.0,-250.0,-70.0);
		} glEnd();
	} glPopMatrix();
	glDisable(GL_TEXTURE_2D);



	// Build Cube
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPushMatrix(); {
		glBegin(GL_QUADS); {
			BuildBox();
		} glEnd();
	} glPopMatrix();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);




	// Boids
	glPushMatrix(); {
//printf("[Boid] x: %f, y: %f, z: %f\n", boids[0].position.x, boids[0].position.y, boids[0].position.z);
		int j;
		float k = 0.5;
		for(j = 0; j < boid_count; j++)
		{
			glPushMatrix(); {
				glTranslatef(boids[j].position.x, boids[j].position.y, boids[j].position.z);
				float yaw_boid = yaw(boids[j].speed);
				float pitch_boid = pitch(boids[j].speed);

				glRotatef(yaw_boid, 0.0, 1.0, 0.0);
				glRotatef(pitch_boid, 1.0, 0.0, 0.0);
				glColor3f(green.r, green.g, green.b);
				glutSolidCone(1.5, 5.0, 20, 20);
				glPushMatrix(); {
					glBegin(GL_TRIANGLES); {
						glVertex3f(0.0, 0.0, 0.0);
						glVertex3f(4.0, 0.0, 0.0);
						glVertex3f(4.25, -1.0, 0.0);
					} glEnd();
					glBegin(GL_TRIANGLES); {
						glVertex3f(0.0, 0.0, 0.0);
						glVertex3f(-4.0, 0.0, 0.0);
						glVertex3f(-4.25, -1.0, 0.0);
					} glEnd();
				} glPopMatrix();
			} glPopMatrix();
		}
	} glPopMatrix();

	// Obstacle
	glPushMatrix(); {
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture1);
		gluSphere( sphere, 10, 16, 16);
		glDisable(GL_TEXTURE_2D);
	} glPopMatrix();


	glEnable(GL_TEXTURE_2D);

	glFlush();
	glutSwapBuffers();
	glutTimerFunc(100, move, 0);
}

void reshape (int w, int h)
{
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, (GLfloat)w/(GLfloat)h, 0.5, 300.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard (unsigned char key, int x, int y)
{
	switch(key) {
		case 'x':
			boids[0].position.x += 0.01;
			break;
		case 'X':
			boids[0].position.x -= 0.01;
			break;
		case 'y':
			boids[0].position.y += 0.01;
			break;
		case 'Y':
			boids[0].position.y -= 0.01;
			break;
		case 'z':
			boids[0].position.z += 0.01;
			break;
		case 'Z':
			boids[0].position.z -= 0.01;
			break;
		case '\t':
			if (boids_B > 0)
				boids_B = 0;
			else
				boids_B = 4000;
			break;
		default:
			break;
	}
}


int main(int argc, char** argv)
{
	srand((time(NULL)));
	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1150, 850);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow("Stephen Shaw - Boids");

	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;
}
