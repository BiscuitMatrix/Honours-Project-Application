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
	void Update(std::vector<boid> *enemy_boids, std::vector<resource> *food, float frame_time);
	void CleanUp();

	void RunBoidsAlgorithm(float frame_time);

	void PhysicsCalculations(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time);

	int GetFlockSize() { return flock_size_; }

	std::vector<boid> boids_;
	

private:
	// References to other entities:
	std::vector<resource> *resources_;
	std::vector<boid> *enemy_boids_;

	gef::Vector2 Cohesion(gef::Vector2, gef::Vector2, int);
	gef::Vector2 Alignment(gef::Vector2, gef::Vector2, gef::Vector2, int);
	gef::Vector2 Separation(gef::Vector2, gef::Vector2, int);
	gef::Vector2 FoodAttraction(std::vector<boid>::iterator);
	gef::Vector2 FlockAvoidance(std::vector<boid>::iterator);

	bool CollisionDetection(float combined_radii_length, float shortest_distance_sqr);

	int flock_size_;

	float flock_health_;

	static float food_detection_sqr_;
	static float interaction_distance_sqr_;

	// Limits
	static float max_force_, max_speed_;
	static float bound_x_, bound_y_;

	gef::Platform& platform_;
};

