#include "flock.h"

float flock::max_speed_ = 500.0f;
float flock::max_force_ = 100.0f;

flock::flock(gef::Platform& platform) :
	platform_(platform)
{
}

flock::~flock()
{
}


void flock::Initialise(gef::Vector2 flock_centre, int flock_size)
{
	for (int i = 0; i < flock_size; i++)
	{
		desired_separation_ = 5.0f;
		interaction_distance_ = 50.0f;

		// Reynolds Weights
		sep_wgt_ = 5.5f, coh_wgt_ = 1.5f, ali_wgt_ = 1.5f;

		boid boid_(platform_);
		boid_.Initialise();

		#pragma region boid_placement
		float x = ((float)i / 10.0f) * 3.14159f;
		float y = ((float)i / 10.0f) * 3.14159f;

		gef::Vector2 pos;

		// Luke! Use the rounding error ~~~~~~~% *poof*
		int mult = i / 10;
		// Generate the concentric cirles of boids
		if (mult > 0)
		{
			pos = gef::Vector2( flock_centre.x + ((float)(mult + 1))*sin(x), flock_centre.y + ((float)(mult + 1))*cos(y) );
		}
		else
		{
			pos = gef::Vector2(flock_centre.x + sin(x), flock_centre.y + cos(y));
		}
		
		boid_.SetTranslation(pos);
		boid_.GetMeshInstance()->set_transform(boid_.GetTranslation());
		#pragma endregion

		boids_.push_back(boid_);
	}
}

void flock::Update(float frame_time)
{
	// Run boids algorithm
	//boid::RunBoidsAlgorithm(&boids_, frame_time);
	RunBoidsAlgorithm(frame_time);

	// Run the Genetic Algorithm

}

void flock::RunBoidsAlgorithm(float frame_time)
{
	// Reynolds Forces
	gef::Vector2 separation_(0.0f, 0.0f), cohesion_(0.0f, 0.0f), alignment_(0.0f, 0.0f);
	// Reynolds Counters
	int sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0;


	for (std::vector<boid>::iterator iterator_ = boids_.begin(); iterator_ != boids_.end(); iterator_++)
	{
		#pragma region Reset_Values 
		// Reset the vectors for separation cohesion and alignment for this boid to zero
		separation_.Reset(), cohesion_.Reset(), alignment_.Reset();
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0;
		#pragma endregion

		// For every other boid in the flock
		for (std::vector<boid>::iterator iterator_2_ = boids_.begin(); iterator_2_ != boids_.end(); iterator_2_++)
		{
			// 0.1.1. Check that we are not calculating against itself
			if (iterator_ != iterator_2_)
			{
				// 0. Find the Euclidean distance between the two boids
				float distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
				// 0.1. If the positions are close enough to interact
				if (distance < interaction_distance_)
				{
					// 1. Separation
					if (distance < desired_separation_)
					{
						// 1.1. Take the difference in position
						gef::Vector2 difference;
						difference = (iterator_->GetCurrPos() - iterator_2_->GetCurrPos());
						// 1.2. Find the unit vector (or v hat) values in x and y 
						difference.Normalise();
						// 1.3. Weight by the distance between the two
						difference /= distance;
						// 1.4. Add this difference to the separation value
						separation_ += difference;
						// 1.5. Increase the counter to use for division later
						sep_counter_++;
					}

					// 2. Cohesion
					// 2.1. Add the position of the neighbour to the sum of all neighbours positions
					cohesion_ += iterator_2_->GetCurrPos();
					// 2.2. increase the counter to use for division later
					coh_counter_++;

					// 3. Alignment
					// 3.1. add the velocity of the neighbour to the sum of all neighbours velocities
					alignment_ += iterator_2_->GetCurrPos();
					// 3.2. increase the counter to for division later
					ali_counter_++;
				}
			}
		}

		// 1. Separation Part 2
		if (sep_counter_ > 1)
		{
			// Calculate the mean of the acted forces
			separation_ /= (float)sep_counter_;
		}

		if (separation_.Length() > 0.0f)
		{
			// Implement Reynolds: Steering = Desired - Velocity
			separation_.Normalise();
			// Multiply the force by the max speed variable
			separation_ *= max_speed_;
			// Minus the current velocity of the boid
			separation_ -= iterator_->GetCurrVel();
			// limit the magnitude of the sep vector
			separation_.Limit(max_force_);
		}


		// 2. Cohesion Part 2
		if (coh_counter_ > 1)
		{
			// Calculate the mean of the acted forces
			cohesion_ /= (float)coh_counter_;
		}
		else
		{
			cohesion_ = gef::Vector2(0.0f, 0.0f);
		}

		// 3. Alignment Part 2
		if (ali_counter_ > 0)
		{
			if (ali_counter_ > 1)
			{
				// Calculate the mean of the acted forces
				alignment_ /= (float)ali_counter_;
			}
			// Implement Reynolds: Steering = Desired - Velocity
			alignment_.Normalise();
			alignment_ *= max_speed_;
			alignment_ -= iterator_->GetCurrVel();
			alignment_.Limit(max_force_);
		}
		else
		{
			alignment_ = gef::Vector2(0.0f, 0.0f);
		}

		// Weight the final value
		separation_ *= sep_wgt_;
		cohesion_ *= coh_wgt_;
		alignment_ *= ali_wgt_;

		// Add the accelerative forces together
		gef::Vector2 acceleration = separation_ + cohesion_ + alignment_;
		iterator_->SetAccel(acceleration);

		// v = u + at
		gef::Vector2 velocity = (iterator_->GetPrevVel()) + (acceleration * frame_time);
		iterator_->SetCurrVel(velocity);

		// s = ut + 1/2(a(t^2))
		gef::Vector2 displacement = (iterator_->GetPrevVel() * frame_time) + ((acceleration*pow(frame_time, 2)) / 2.0f);
		iterator_->SetDisplacement(displacement);

		// x1 = x0 + s
		gef::Vector2 new_pos = iterator_->GetCurrPos() + iterator_->GetDisplacement();
		iterator_->SetCurrPos(new_pos);

		//iterator_->WrapAround(55.0f, 30.0f);
		iterator_->Bounds(55.0f, 30.0f);

		// Set the boids prev values so we can reference the values of the previous frame in the next frame
		iterator_->SetPrevVel(iterator_->GetCurrVel());
		iterator_->SetPrevPos(iterator_->GetCurrPos());

		// Update the translation of the object within the scene
		iterator_->SetTranslation(iterator_->GetCurrPos());

		gef::Matrix44 final_transform = iterator_->GetScale() * iterator_->GetRotation() * iterator_->GetTranslation();
		iterator_->GetCube()->set_transform(final_transform);
	}
}

void flock::PhysicsCalculations()
{

}

void flock::CleanUp()
{
}
