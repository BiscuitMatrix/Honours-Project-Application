#pragma once

#include <vector>

#include "boid.h"

class flock
{
public:
	flock(gef::Platform& platform);
	~flock();

	void Initialise(int flock_size);
	void Update(float frame_time);
	void CleanUp();

	float CalcDist();

	int GetFlockSize() { return flock_size_; }

	std::vector<boid>* boids_;
	std::vector<boid>::iterator iterator_;
	std::vector<boid>::iterator iterator_2_;

private:
	int flock_size_;

	float desired_separation_;

	gef::Platform& platform_;
};

