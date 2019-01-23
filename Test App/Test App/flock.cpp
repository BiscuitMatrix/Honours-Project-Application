#include "flock.h"



flock::flock(gef::Platform& platform) :
	platform_(platform)
{
}

flock::~flock()
{
}


void flock::Initialise(int flock_size)
{
	boids_ = &std::vector<boid>();
	boids_->begin();

	for (int i = 0; i < flock_size; i++)
	{
		boid* boid_ = new boid(platform_);
		boid_->Initialise();
		boids_->push_back(*boid_);
	}
}

void flock::Update(float frame_time)
{
	// Run boids algoirthm
	RunBoidsAlgorithm();

	// Run the Genetic Algorithm

}

void flock::RunBoidsAlgorithm()
{
	
}

void flock::CleanUp()
{
}
