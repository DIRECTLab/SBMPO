#include "collision.h"

/** Collision Section */
void OBB::orient(double orientation) {
	u[0][0] = cos(orientation);
	u[0][1] = sin(orientation);
	u[0][2] = 0;
	u[1][0] = -sin(orientation);
	u[1][1] = cos(orientation);
	u[1][2] = 0;
	u[2][0] = 0;
	u[2][1] = 0;
	u[2][2] = 1;
}

void OBB::translate(vec3 center) {
	c = center;
}

void OBB::update(State state, int dof) {
	vec3 center;
	center[0] = state[0];
	center[1] = state[1];
	if (dof == 3) {
		center[2] = state[2];
		translate(center);
		orient(state[3]);
	} else {
		center[2] = 0.0;
		translate(center);
		orient(state[2]);
	}
}

void OBB::ClosestPointOBB(vec3 p, vec3 &q) {
	vec3 d;
	d = p - c;
	q = c;
	for (int i = 0; i < 2; ++i) {
		double dist = d * u[i];
		if (dist > e[i])  dist = e[i];
		if (dist < -e[i]) dist = -e[i];
		q += dist * u[i];
	}
}

void OBB::resize(double length, double width) {
	e[0] = length / 2.0;
	e[1] = width / 2.0;
	e[2] = 0.0;
}

bool OBB::IntersectCircle(State state, Obstacle obs) {
	vec3 p;
	vec3 center;
	double r = obs[2];
	center[0] = obs[0];
	center[1] = obs[1];
	center[2] = 0.0;
	update(state, 2);
	ClosestPointOBB(center, p);
	vec3 v;
	v = p - center;
	return v * v <= r * r;
}

bool OBB::Intersect(State state, Obstacle obs, int dof) {
	vec3 p;
	vec3 center;
	double r;
	center[0] = obs[0];
	center[1] = obs[1];
	if (dof == 3) {
		center[2] = obs[2];
		r = obs[3];
	} else {
		center[2] = 0.0;
		r = obs[2];
	}
	update(state, dof);
	ClosestPointOBB(center, p);
	vec3 v;
	v = p - center;
	return v * v <= r * r;
}
