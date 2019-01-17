#pragma once

#include <vector>

#include "boid.h"

class flock
{
public:
	flock(gef::Platform& platform);
	~flock();

	void Initialise(int flock_size);
	void Update();
	void CleanUp();

	int GetFlockSize() { return flock_size_; }

	std::vector<boid> boids_;
	std::vector<boid>::iterator iterator_;

private:
	//boid* boid_;
	int flock_size_;

	gef::Platform& platform_;
};

