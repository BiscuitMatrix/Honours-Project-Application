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
	boid_ref_ = new boid(platform_);

	boids_ = new std::vector<boid>();
	boids_->begin();

	for (int i = 0; i < flock_size; i++)
	{
		boid* boid_ = new boid(platform_);
		boid_->Initialise();
		boids_->push_back(*boid_);
	}
	int size = boids_->size();
}

void flock::Update(float frame_time)
{
	// Run boids algorithm
	boid_ref_->RunBoidsAlgorithm(boids_, frame_time);
	
	// Run the Genetic Algorithm

}

void flock::RunBoidsAlgorithm()
{
	
}

void flock::CleanUp()
{
}
