#pragma once
#include <vector>

// radius in pixels

class particle
{
	public:

		float radius;
		float pos_x, pos_y, vel_x_, vel_y_, acl_x_, acl_y_;
		int mass;

		particle(float x, float y, float radius, int mass);

		particle(const particle &p2) {
			pos_x = p2.pos_x;
			pos_y = p2.pos_y;
			radius = p2.radius;
			vel_x_ = p2.vel_x_;
			vel_y_ = p2.vel_y_;
			acl_x_ = p2.acl_x_;
			acl_y_ = p2.acl_y_;
			mass = p2.mass;
		};

	private:

};

