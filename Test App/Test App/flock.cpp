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
//	boid_ref_ = new boid(platform_);
//	boid_ref_->Initialise();

//	boids_ = new std::vector<boid>();
//	boids_->begin();

	for (int i = 0; i < flock_size; i++)
	{
		//boid* boid_ = new boid(platform_);
		// boid_->Initialise();
		boid boid_(platform_);
		boid_.Initialise();
		float x = ((float)i / 5.0f) * 3.14159f;
		float y = ((float)i / 5.0f) * 3.14159f;
		gef::Vector4 pos;
		if (i <= 10)
		{
			pos = gef::Vector4(sin(x), 0.0f, cos(y));
		}
		else if (i > 10 && i <= 20)
		{
			pos = gef::Vector4(2.0f*sin(x), 0.0f, 2.0f*cos(y));
		}
		else if (i > 20 && i <= 30)
		{
			pos = gef::Vector4(3.0f*sin(x), 0.0f, 3.0f*cos(y));
		}
		else if (i > 30 && i <= 40)
		{
			pos = gef::Vector4(4.0f*sin(x), 0.0f, 4.0f*cos(y));
		}
		else if (i > 40 && i <= 50)
		{
			pos = gef::Vector4(5.0f*sin(x), 0.0f, 5.0f*cos(y));
		}
//		boid_->SetTranslation(pos);
//		boid_->GetMeshInstance()->set_transform(boid_->GetTranslation());
//		boids_->push_back(*boid_);
		boid_.SetTranslation(pos);
		boid_.GetMeshInstance()->set_transform(boid_.GetTranslation());
		boids_.push_back(boid_);
	}
	//int size = boids_->size();
}

void flock::Update(float frame_time)
{
	// Run boids algorithm
//	boid_ref_->RunBoidsAlgorithm(boids_, frame_time);
	boid::RunBoidsAlgorithm(&boids_, frame_time);

	// Run the Genetic Algorithm

}

void flock::RunBoidsAlgorithm()
{
	// Reynolds Forces
	gef::Vector2 separation_(0.0f, 0.0f), cohesion_(0.0f, 0.0f), alignment_(0.0f, 0.0f);
	// Reynolds Counters
	int sep_counter_, ali_counter_, coh_counter_;


	for (std::vector<boid>::iterator iterator_ = boids_.begin(); iterator_ != boids_.end(); iterator_++)
	{
		// Reset the vectors for separation cohesion and alignment for this boid to zero
		separation_.Reset(), cohesion_.Reset(), alignment_.Reset();

		// Define vectors to take the sume of positions and velocities of neighbouring boids respectively
		gef::Vector2 sum_of_positions(0, 0), sum_of_velocities(0, 0);
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0;

		// For every other boid in the flock
		for (std::vector<boid>::iterator iterator_2_ = boids_.begin(); iterator_2_ != boids_.end(); iterator_2_++)
		{
			// 0.1.1. Check that we are not calculating against itself
			if (iterator_ != iterator_2_)
			{
				// 0. Find the Euclidean distance between the two boids
				float distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
				// 0.1. If the positions are close enough to interact
				if (distance < 30.0f)
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
			separation_ -= iterator_.curr_vel_;
			// limit the magnitude of the sep vector
			iterator_->separation_->Limit(max_force_);
		}


		// 2. Cohesion Part 2
		if (coh_counter_ > 1)
		{
			// Calculate the mean of the acted forces
			*iterator_->cohesion_ /= (float)coh_counter_;
		}
		else
		{
			*iterator_->cohesion_ = gef::Vector2(0.0f, 0.0f);
		}

		// 3. Alignment Part 2
		if (ali_counter_ > 0)
		{
			if (ali_counter_ > 1)
			{
				// Calculate the mean of the acted forces
				*iterator_->alignment_ /= (float)ali_counter_;
			}
			// Implement Reynolds: Steering = Desired - Velocity
			iterator_->alignment_->Normalise();
			*iterator_->alignment_ *= max_speed_;
			*iterator_->alignment_ -= iterator_->curr_vel_;
			iterator_->alignment_->Limit(max_force_);
		}
		else
		{
			*iterator_->alignment_ = gef::Vector2(0.0f, 0.0f);
		}

		// Weight the final value
		*iterator_->separation_ *= iterator_->sep_wgt_;
		*iterator_->cohesion_ *= iterator_->coh_wgt_;
		*iterator_->alignment_ *= iterator_->ali_wgt_;

		// Add the accelerative forces together
		iterator_->accel_ = *iterator_->separation_ + *iterator_->cohesion_ + *iterator_->alignment_;

		// v = u + at
		iterator_->curr_vel_ = (iterator_->prev_vel_) + (iterator_->accel_ * frame_time);

		// s = ut + 1/2(a(t^2))
		iterator_->displacement_ = (iterator_->prev_vel_ * frame_time) + ((iterator_->accel_*pow(frame_time, 2)) / 2.0f);

		iterator_->curr_pos_ += iterator_->displacement_;

		//iterator_->WrapAround(30.0f, 30.0f);
		iterator_->Bounds(30.0f, 30.0f);

		// Set the boids prev values so we can reference the values of the previous frame in the next frame
		iterator_->prev_vel_ = iterator_->curr_vel_;
		iterator_->prev_pos_ = iterator_->curr_pos_;

		gef::Vector4 pos = gef::Vector4(iterator_->curr_pos_.x, 0.0f, iterator_->curr_pos_.y);
		iterator_->translation_.SetTranslation(pos);

		gef::Matrix44 final_transform = iterator_->scale_ * iterator_->rotation_ * iterator_->translation_;
		//gef::Matrix44 final_transform = iterator_->translation_;
		iterator_->cube_->set_transform(final_transform);

		//iterator_->UpdatePosition(frame_time);
	}
}

void flock::CleanUp()
{
}
