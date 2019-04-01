#pragma once



class genetic_algorithm
{
public:
	genetic_algorithm();
	~genetic_algorithm();

	float fitness();

private:
	float boid_health_;
	float flock_health_;
	float competing_flock_health_;
};

