#pragma once
#include "field.h"
#include <ctime>
#include <iostream>
#include <math.h>
#include <fstream>
#include <chrono>
#include <string>
#include <math.h>

using namespace std;
using namespace std::chrono;

void display();
void spawn(int x, int y); //Changed function to spawn to allow spawning without mouse clicks.
void keyboard(unsigned char key, int x, int y);
void tick(int i);


//Added to easily change the number of particles, which impacts the spacing between the particles shown in particleSpacing.
float numParticles = 8000; //Change to set the number of particles spawned, must be 1-10000 and be a multiple of 5.
float numParticlesSqrt = sqrt(numParticles);
int particleSpacing = 5 / (numParticlesSqrt / 200);
double lastTime = GetTickCount();
int nbFrames = 0;

std::vector<duration<double>> times;
field *space = nullptr;

//Exit function to write times to file.
int exiting() 
{
	//Create/Open .csv file for storing results.
	ofstream out;
	string outFile = ("results-OpenMP.csv"); //std::to_string(numParticles)
	out.open(outFile, std::ios::app);
	
	//Write to .csv file.
	for each(duration<double> i in times)
	{
		out << i.count() << endl;
	}

	//Close stream.
	out.close();

	return 0;
}

int main(int argc, char **argv)
{
	const int window_size = 1000;

	space = new field(window_size);

	/*                  CREATE WINDOW                                                   */
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(window_size, window_size);
	glutInitWindowPosition(50, 50);
	auto win = glutCreateWindow("Gravity_Simulation Application");

	/*                  SET UP MATRICES													 */
	glClearColor(0, 0,  0, 1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, window_size, window_size, 0, 0, 1);
	
	glutDisplayFunc(display);
	int counter = 0;
	//Loop to spawn all the particles on load.
	for (int i = 0; i < numParticlesSqrt; i++)
	{
		for (int j = 0; j < numParticlesSqrt; j++)
		{
			if (counter < numParticles)
			{ 
				spawn(5 + (i * particleSpacing), 5 + (j * particleSpacing));
				counter++;
			}
		}
	}

	glutKeyboardFunc(keyboard);
	glClear(GL_COLOR_BUFFER_BIT);
	glFlush();

	tick(0);

	glutMainLoop();

	delete space;	
}

void display()
{
	space->display();
}

void spawn(int x, int y)
{
	space->spawn_particle(x, y);
}

void keyboard(unsigned char key, int x, int y)
{
	space->keyboard_input(key, x, y);
}

void tick(int i)
{
	auto start = system_clock::now();

    space->update_particles();

	auto end = system_clock::now();
	times.push_back(end - start);

	if (std::size(times) == 100)
	{
		std::exit(exiting());
	}
	
	//glutTimerFunc(33, tick, 0);
	tick(0);
}