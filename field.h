#pragma once

#include "Particle.h"
#include "gl/freeglut.h"
#include <cmath>
#include <vector>


class field
{
	private:
		explicit field(const int window_size);
		struct cell
		{
			int length_px;
			float gravity = 0;
			bool is_occupied = false;
		};

		std::vector<particle> particles_;
		std::vector<particle> particlesThread;

		cell cells_[100][100];
		int window_size_;

	public:
		static const int window_size = 1000;
		inline static field* get()
		{
			static field* instance(new field(window_size));
			return instance;
		}

		void spawn_particle(int x, int y);
		void display();
		void keyboard_input(unsigned char key, int x, int y);
		void add_space_curvature(int x, int y, int mass);
		void update_particles_loop(std::vector<particle> &particles, int count, int number);
		void update_particles();
		void find_mass_centers(int x, int y, int *x_origin, int *y_origin, int *xf, int *yf, int *xc, int *yc);
};

