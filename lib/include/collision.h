#pragma once

#include "definitions.h"
#include "vec3.h"

/// @brief oriented-bounding box collision detection
class OBB { // Oriented-Bounding Box
public:
	vec3 c;
	vec3 e;
	vec3 u[3];

	void orient(double orientation);
	void translate(vec3 center);
	void update(State state, int dof);
	void ClosestPointOBB(vec3 p, vec3 &q);
	void resize(double length, double width);
	bool IntersectCircle(State state, Obstacle obs);
	bool Intersect(State state, Obstacle obs, int dof);
};
