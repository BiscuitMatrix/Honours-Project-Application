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
	for (iterator_ = boids_->begin(); iterator_ != boids_->end(); iterator_++)
	{
		// Define the vectors for separation, cohesion and alignment
		Vector2 sep, coh, ali;

		// Define vectors to take the sume of positions and velocities of neighbouring boids respectively
		Vector2 sum_of_positions(0,0), sum_of_velocities(0, 0);
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		int sep_counter, ali_counter, coh_counter = 0;

		// For every other boid in the flock
		for (iterator_2_ = boids_->begin(); iterator_2_ != boids_->end(); iterator_2_++)
		{
			// 0. Take the difference position of the two boids
			float dist = abs(CalcDist());
			// 0.1. If the positions are close enough to interact
			if (dist < 50.0f) 
			{
				// 0.1.1. Check that we are not calculating against itself
				if (iterator_ != iterator_2_)
				{
					// 1. Separation
					if (dist < desired_separation_)
					{
						// 1.1. Calculate the magnitude of the vector pointing away from the neighbour

						// 1.2. Calculate the direction of the vector pointing away from the neighbour

						// 1.3. 
					}
					

					// 2. Cohesion
					// 2.1. Add the position of the neighbour to the sum of all neighbours positions
					//sum_of_positions += neighbour_pos;

					// 2.2. increase the counter to use for division later
					coh_counter++;


					// 3. Alignment
					// 3.1. add the velocity of the neighbour to the sum of all neighbours velocities
					//sum_of_velocities += neighbour_vel;

					// 3.2. increase the counter to for division later
					ali_counter++;

				}
			}

			
		}

		// Update the boid now all the information surrounding it has been updated
		iterator_->Update(frame_time);
	}
}

float flock::CalcDist()
{
	float result;
	


	return result;
}

void flock::CleanUp()
{
}
