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
		gef::Vector2 sep(0.0f, 0.0f), coh(0.0f, 0.0f), ali(0.0f, 0.0f);

		// Define vectors to take the sume of positions and velocities of neighbouring boids respectively
		gef::Vector2 sum_of_positions(0,0), sum_of_velocities(0, 0);
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		int sep_counter = 0, ali_counter = 0, coh_counter = 0;

		// For every other boid in the flock
		for (iterator_2_ = boids_->begin(); iterator_2_ != boids_->end(); iterator_2_++)
		{
			// 0. Find the Euclidean distance between the two boids
			float distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
			// 0.1. If the positions are close enough to interact
			if (distance < 50.0f) 
			{
				// 0.1.1. Check that we are not calculating against itself
				if (iterator_ != iterator_2_)
				{
					// 1. Separation
					if (distance < desired_separation_)
					{
						// 1.1. Take the difference in position
						gef::Vector2 difference;
						difference = (iterator_2_->GetCurrPos() - iterator_->GetCurrPos());
						// 1.2. Find the unit vector (or v hat) values in x and y 
						difference.Normalise();
						// 1.3. Weight by the distance between the two
						difference /= distance;
						// 1.4. Add this difference to the separation value
						sep += difference;
						// 1.5. Increase the counter to use for division later
						sep_counter++;
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

		// 1. Separation Part 2
		if (sep_counter > 0)
		{
			// Calculate the mean of the acted forces
			sep /= sep_counter;
		}

		if (sep.Length() < 0.0f)
		{
			// 1.1 Set the magnitude of the vector to "maxspeed"

			// 1.2. Implement Reynolds: Steering = Desired - Velocity
			sep.Normalise();
			//sep *= maxspeed;
			//sep -= velocity;
			// limit the magnitude of the sep vector
		}

		// 2. Cohesion Part 2


		// 3. Alignment Part 2


		// Update the boid now all the information surrounding it has been updated
		iterator_->Update(frame_time);
	}
}

void flock::CleanUp()
{
}
