#include "field.h"
#include <chrono>
#include <iostream>
#include <omp.h>
#include <thread>

using namespace std;

//Removed black hole mass.
#define M_PI 3.14159265
#define RADIUS 5
#define MASS 2
#define DEBUG 0

bool debug = false;
bool freeze = false;

field::field(int window_size)
{

	while(window_size % (RADIUS * 2) != 0)
	{
		window_size++;
	}

	const int cell_amt = window_size / (RADIUS * 2);
	cells_ = new cell*[cell_amt];
	for(int i = 0; i < cell_amt; ++i)
		cells_[i] = new cell[cell_amt];

	window_size_ = window_size;
}


field::~field()
{
	const int cell_amt = window_size_ / (RADIUS * 2);

	for(int i = 0; i < cell_amt; i++)
	{
		delete[] cells_[i];
	}

	delete [] cells_;
}

//Changed to spawn particles soley on position and removed black holes.
void field::spawn_particle(int x, int y)
{
	if (debug == false && freeze == false) {
		int x_origin, y_origin;
		int x_f, y_f, x_c, y_c;
		find_mass_centers(x, y, &x_origin, &y_origin, &x_f, &y_f, &x_c, &y_c);

		if (cells_[(int)y_origin][(int)x_origin].is_occupied) {
#if DEBUG == 1 
		std::cout << "Space occupied" << std::endl;

#endif
			return;
		}

		particles_.emplace_back(x, y, RADIUS, MASS);

#if DEBUG == 1 
	std::cout << "Particle created" << std::endl;

#endif
		add_space_curvature(x_origin, y_origin, MASS);

		cells_[y_origin][x_origin].is_occupied = true;

		return;
	} 
}

//Removed for loop used to display black holes.
void field::display()
{
	glClear(GL_COLOR_BUFFER_BIT);


	for(auto &p : particles_)
	{
		float pos_x = p.pos_x;
		float pos_y = p.pos_y;
		float radius = p.radius;

		glColor3f(1, 1, 1);
		glBegin(GL_POLYGON);
		for (float a = 0; a < 2 * M_PI; a += 0.1) {
			glVertex2f(radius * cos(a) + pos_x, radius * sin(a) + pos_y);
		}
		glEnd();
	}
	
	glFlush();
}

void field::keyboard_input(unsigned char key, int x, int y)
{
	if(key == 'g')
	{
		freeze = !freeze;
		debug = false;
		std::cout << "g pressed" << std::endl;
	}
	else if (key == 'd')
	{
		debug = !debug;
		freeze = false;
		std::cout << "d pressed" << std::endl;
	}

	
}

void field::add_space_curvature(int x, int y, int mass)
{

	cells_[y][x].is_occupied = !cells_[y][x].is_occupied;
	for (int grow = 1; grow < 8; grow++)
	{
		for(int i = (-1 * grow) + y ; i <= grow + y; i++)
		{
			for (int j = (-1 * grow) + x; j <= grow + x; j++)
			{
				if ( i < 0 | i  > (window_size_ / (RADIUS * 2)) - 1) break;
				if (j < 0) continue;
				if (j > (window_size_ / (RADIUS * 2)) - 1) break;

				if (i == y && j == x) cells_[i][j].gravity += mass;
				else
				{
					cells_[i][j].gravity += (float) mass / (grow * 5);

#if DEBUG == 1 
					std::cout << "gravity at (" << i << ", " << j << ") is " << (float) cells_[i][j].gravity << std::endl;
					
#endif
				} 
			}
		}
	}
}

// Function that is run constantly O(N)
void field::update_particles()
{
	if (debug | freeze) return;

	int numThreads = thread::hardware_concurrency();

#pragma omp parallel for num_threads(numThreads)
	for (int i = 0; i < size(particles_); i++)
	{
		int prev_center_x;
		int prev_center_y;

		// O(1)
		find_mass_centers(particles_[i].pos_x, particles_[i].pos_y, &prev_center_x, &prev_center_y, nullptr, nullptr, nullptr, nullptr);

		// O(1)
		add_space_curvature(prev_center_x, prev_center_y, particles_[i].mass * (-1));

		// update postion from velocity
		if (!(particles_[i].pos_x + particles_[i].vel_x_ >= window_size_ | particles_[i].pos_x + particles_[i].vel_x_  <= 0)) {
			particles_[i].pos_x += particles_[i].vel_x_;
		} else
		{
			particles_[i].vel_x_ = -particles_[i].vel_x_;
			particles_[i].acl_x_ = 0;
		}
		if (!(particles_[i].pos_y + particles_[i].vel_y_  >= window_size_ | particles_[i].pos_y + particles_[i].vel_y_ <= 0)) {
			particles_[i].pos_y += particles_[i].vel_y_;
		} else
		{
			particles_[i].vel_y_ = -particles_[i].vel_y_;
			particles_[i].acl_y_ = 0;
		}
		int updated_center_x;
		int updated_center_y;
		// O(1)
		find_mass_centers(particles_[i].pos_x, particles_[i].pos_y, &updated_center_x, &updated_center_y, nullptr, nullptr, nullptr, nullptr);
		// O(1)
		add_space_curvature(updated_center_x, updated_center_y, particles_[i].mass);
		// update velocity from acceleration
		if (!(particles_[i].vel_x_ + particles_[i].acl_x_ > 3.5 | particles_[i].vel_x_ + particles_[i].acl_x_ < -3.5)) {
			particles_[i].vel_x_ += particles_[i].acl_x_;
		} 
		if (!(particles_[i].vel_y_ + particles_[i].acl_y_ > 3.5 | particles_[i].vel_y_ + particles_[i].acl_y_ < -3.5)) {
			particles_[i].vel_y_ += particles_[i].acl_y_;

		}

		// update acceleration from curvature

		int x_origin, y_origin;
		float left_acl, right_acl, up_acl, down_acl;
		left_acl = right_acl = up_acl = down_acl = 0;
		bool left_z, right_z, up_z, down_z;
		left_z = right_z = up_z = down_z = false;
		int x_f, y_f, x_c, y_c;
		// O(1)
		find_mass_centers(particles_[i].pos_x, particles_[i].pos_y, &x_origin, &y_origin, &x_f, &y_f, &x_c, &y_c);

		if (x_origin - 1 < 0) {
			left_z = true;
		}
		if (x_origin + 1 > ((window_size_ / (RADIUS * 2)) - 1))
		{
			right_z = true;
		}
		if (y_origin - 1 < 0) {
			up_z = true;
		}
		if (y_origin + 1 > ((window_size_ / (RADIUS * 2)) - 1))
		{
			down_z = true;
		}
		if (!left_z) {
			left_acl += cells_[y_origin][x_origin - 1].gravity;
			if (!down_z) left_acl += cells_[y_origin + 1][x_origin - 1].gravity;
			if (!up_z) left_acl += cells_[y_origin - 1][x_origin - 1].gravity;
		}

		if (!right_z)
		{
			right_acl += cells_[y_origin][x_origin + 1].gravity;
			if (!down_z) right_acl += cells_[y_origin + 1][x_origin + 1].gravity;
			if (!up_z) right_acl += cells_[y_origin - 1][x_origin + 1].gravity;
		}

		if(!down_z)
		{
			down_acl += cells_[y_origin + 1][x_origin].gravity;
			if (!right_z) down_acl += cells_[y_origin + 1][ x_origin + 1].gravity;
			if (!left_z) down_acl += cells_[ y_origin + 1][ x_origin - 1].gravity;
		}
		if (!up_z)
		{
			up_acl += cells_[y_origin - 1][x_origin].gravity;
			if (!right_z) up_acl += cells_[y_origin - 1][x_origin + 1].gravity;
			if (!left_z) up_acl += cells_[y_origin - 1][x_origin - 1].gravity;
		}

		if (particles_[i].acl_x_ + right_acl - left_acl > 1) {
			particles_[i].acl_x_ = 1;
			continue;
		}
		else if (particles_[i].acl_x_ + right_acl - left_acl < -1) {
			particles_[i].acl_x_ = -1;
			continue;
		}
		if (particles_[i].acl_y_ + down_acl - up_acl > 1) {
			particles_[i].acl_y_ = 1;
			continue;
		} else if (particles_[i].acl_y_ + down_acl - up_acl < -1)
		{
			particles_[i].acl_y_ = -1;
			continue;
		}
		particles_[i].acl_x_ += right_acl - left_acl;
		particles_[i].acl_y_ += down_acl - up_acl;
	}

	display();

}

void field::find_mass_centers(int x, int y, int *x_origin, int *y_origin, int *xf, int *yf, int *xc, int *yc)
{
	int half = RADIUS;

	if(x % (RADIUS * 2) == 0)
	{
		*x_origin = (x / (RADIUS * 2));
		if (*x_origin >= (window_size_ / (RADIUS * 2)))
		{
			*x_origin = (window_size_ / (RADIUS * 2)) - 1;
		}
		else if (*x_origin <= 0)
		{
			*x_origin = 1;
		}
	}
	else if(x % (RADIUS * 2) < half)
	{
		*x_origin = x / (RADIUS * 2);
		if(*x_origin >= (window_size_ / (RADIUS * 2)))
		{
			*x_origin = (window_size_ / (RADIUS * 2)) - 1;
		} else if (*x_origin <= 0)
		{
			*x_origin = 1;
		}
	} else
	{
		*x_origin = (x / (RADIUS * 2)) + 1;
		if (*x_origin >= (window_size_ / (RADIUS * 2)))
		{
			*x_origin = (window_size_ / (RADIUS * 2)) - 1;
		}
		else if (*x_origin <= 0)
		{
			*x_origin = 1;
		}
	}

	if (y % (RADIUS * 2) == 0)
	{
		*y_origin = (y / (RADIUS * 2));
		if (*y_origin >= (window_size_ / (RADIUS * 2)))
		{
			*y_origin = (window_size_ / (RADIUS * 2)) - 1;
		}
		else if (*y_origin <= 0)
		{
			*y_origin = 1;
		}
	}
	else if (y % (RADIUS * 2) < half)
	{
		*y_origin = y / (RADIUS * 2);
		if (*y_origin >= (window_size_ / (RADIUS * 2)))
		{
			*y_origin = (window_size_ / (RADIUS * 2)) - 1;
		}
		else if (*y_origin <= 0)
		{
			*y_origin = 1;
		}
	}
	else
	{
		*y_origin = (y / (RADIUS * 2)) + 1;
		if (*y_origin >= (window_size_ / (RADIUS * 2)))
		{
			*y_origin = (window_size_ / (RADIUS * 2)) - 1;
		}
		else if (*y_origin <= 0)
		{
			*y_origin = 1;
		}
	}

}
