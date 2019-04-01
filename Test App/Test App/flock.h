#pragma once

#include <vector>
#include <math.h>
#include <thread>
#include <maths/vector2.h>

#include "boid.h"
#include "food.h"

class flock
{
public:
	flock(gef::Platform& platform);
	~flock();

	void Initialise(gef::Vector2 flock_centre, int flock_size);
	void Update(float frame_time);
	void CleanUp();

	void RunBoidsAlgorithm(float frame_time);

	void PhysicsCalculations(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time);
	gef::Vector2 AvoidBoundary(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time);

	int GetFlockSize() { return flock_size_; }

	std::vector<boid> boids_;
	//std::vector<food> food_;
	//std::vector<boid> enemy_boids_;

private:
	int flock_size_;

	float desired_separation_;
	float food_detection_;
	float interaction_distance_;
	// Limits
	static float max_force_, max_speed_;
	// Flocking Weights
	float sep_wgt_, coh_wgt_, ali_wgt_, feed_wgt_, neut_wgt_, free_wgt_, bnd_wgt_, ob_av_wgt_;

	gef::Platform& platform_;
};

