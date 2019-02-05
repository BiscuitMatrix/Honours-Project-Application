#pragma once

#include <vector>
#include <maths/vector2.h>

#include "boid.h"

class flock
{
public:
	flock(gef::Platform& platform);
	~flock();

	void Initialise(int flock_size);
	void Update(float frame_time);
	void CleanUp();

	void RunBoidsAlgorithm();

	int GetFlockSize() { return flock_size_; }

	std::vector<boid> boids_;

private:
	int flock_size_;

	float desired_separation_;

	gef::Platform& platform_;
};

