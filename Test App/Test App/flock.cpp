#include "flock.h"

float flock::max_speed_ = 5.0f;
float flock::max_force_ = 3.0f;

float flock::bound_x_ = 55.0f;
float flock::bound_y_ = 30.0f;

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
		desired_separation_ = 8.0f;
		food_detection_ = 4.0f;
		interaction_distance_ = 10.0f;

		// Reynolds Weights
		sep_wgt_ = 1.5f, coh_wgt_ = 1.5f, ali_wgt_ = 1.5f;

		boid boid_(platform_);
		boid_.Initialise();

		#pragma region boid_placement
		float x = ((float)i / 5.0f) * 3.14159f;
		float y = ((float)i / 5.0f) * 3.14159f;

		gef::Vector2 pos;

		// Luke! Use the rounding error ~~~~~~~% *poof*
		int mult = i / 10;
		// Generate the concentric cirles of boids
		if (mult > 0)
		{
			pos = gef::Vector2( flock_centre.x + 2*((float)(mult + 1))*sin(x), flock_centre.y + 2*((float)(mult + 1))*cos(y) );
		}
		else
		{
			pos = gef::Vector2(flock_centre.x + 2*sin(x), flock_centre.y + 2*cos(y));
		}
		
		boid_.SetPos(pos);
		boid_.GetMeshInstance()->set_transform(boid_.GetTranslationMatrix());
		#pragma endregion

		boids_.push_back(boid_);
	}
}

void flock::Update(float frame_time)
{
	// Run boids algorithm
	//boid::RunBoidsAlgorithm(&boids_, frame_time); <- This is a good reference, but dont use it in this program
	//RunBoidsAlgorithm1(frame_time);
	RunBoidsAlgorithm2(frame_time);

	// Run the Genetic Algorithm


}

void flock::RunBoidsAlgorithm1(float frame_time)
{
	#pragma region Forces
	// Reynolds Forces
	gef::Vector2 separation_(0.0f, 0.0f), cohesion_(0.0f, 0.0f), alignment_(0.0f, 0.0f);
	// Expanded boids model forces
	gef::Vector2 food_attraction_(0.0f, 0.0f), boundary_avoidance(0.0f,0.0f);
	// Reynolds Counters
	int sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0, food_counter_ = 0;
	#pragma endregion

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
			// Check that we are not calculating against itself
			if (iterator_ != iterator_2_)
			{
				// 0. Find the Euclidean distance between the two boids
				float euclidean_distance = sqrtf(pow((iterator_2_->GetPos().x - iterator_->GetPos().x), 2) + pow((iterator_2_->GetPos().y - iterator_->GetPos().y), 2));
				// 0.1. If the positions are close enough to interact
				if (euclidean_distance < interaction_distance_)
				{
					// 1. Separation
					if (euclidean_distance < desired_separation_)
					{
						// 1.1. Take the difference in position
						gef::Vector2 difference;
						difference = (iterator_->GetPos() - iterator_2_->GetPos());
						// 1.2. Find the unit vector (or v hat) values in x and y 
						difference.Normalise();
						// 1.3. Weight by the distance between the two
						difference /= euclidean_distance;
						// 1.4. Add this difference to the separation value
						separation_ += difference;
						// 1.5. Increase the counter to use for division later
						sep_counter_++;
					}

					// 2. Cohesion
					// 2.1. Add the position of the neighbour to the sum of all neighbours positions
					cohesion_ += iterator_2_->GetPos();
					// 2.2. increase the counter to use for division later
					coh_counter_++;

					// 3. Alignment
					// 3.1. add the velocity of the neighbour to the sum of all neighbours velocities
					alignment_ += iterator_2_->GetPos();
					// 3.2. increase the counter to for division later
					ali_counter_++;
				}
			}
		}

		// For each food item in the scene
		//for (std::vector<food>::iterator iterator_2_ = food_.begin(); iterator_2_ != food_.end(); iterator_2_++)
		//{
		//  // Reset the food force vector
			//food_attraction_.kZero;
		//
		//	// Find the Euclidean distance between the boid and the food resource
		//	float euclidean_distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
		//
		//	// 4. Food attraction
		//	if (euclidean_distance < food_detection_) // Boid would like to know your location
		//	{
		//		// 4.1 Take a vector for the difference in position
		//		gef::Vector2 difference;
		//		difference = (iterator_2_->GetCurrPos() - iterator_->GetCurrPos());
		//
		//
		//		// 4.?. Increase the counter to use for division later
		//		food_counter_++;
		//	}
		//}
		//
		// For each enemy boid in the scene
		//for (std::vector<boid>::iterator iterator_2_ = enemy_boids_.begin(); iterator_2_ != enemy_boids_.end(); iterator_2_++)
		//{
		//	// Find the Euclidean distance between the boid and the enemy boid
		//	float euclidean_distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
		//
			// 5. Inter-Flock Relation
		//
		//}

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
			separation_ -= iterator_->GetVel();
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
			alignment_ -= iterator_->GetVel();
			alignment_.Limit(max_force_);
		}
		else
		{
			alignment_ = gef::Vector2(0.0f, 0.0f);
		}

		// 4. Food Attraction

		// 5. Inter-Flock Relation part 2

		// 6. Boundary Avoidance
		// 6.1. Reset the boundary avoidance force
		boundary_avoidance.Reset();
		// 6.2. 
		
		// 7. Free Movement


		// Weight the final values for each force
		separation_ *= sep_wgt_, cohesion_ *= coh_wgt_, alignment_ *= ali_wgt_;

		// Add the accelerative forces together
		gef::Vector2 acceleration = separation_ + cohesion_ + alignment_;

		// Update Physics
		PhysicsCalculations(iterator_, acceleration, frame_time);
	}
}

void flock::RunBoidsAlgorithm2(float frame_time)
{
	// Reynolds Forces
	gef::Vector2 separation(0.0f, 0.0f), cohesion(0.0f, 0.0f), alignment(0.0f, 0.0f);
	// Reynolds Counters
	//int sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0, food_counter_ = 0;
	gef::Vector2 avg_neighbour_pos(0.0f, 0.0f);
	gef::Vector2 avg_neighbour_vel(0.0f, 0.0f);
	gef::Vector2 closest_neighbour(0.0f, 0.0f);
	float distance_to_neighbour = 0.0f;
	int neighbour_count = 0;

	for (std::vector<boid>::iterator iterator = boids_.begin(); iterator != boids_.end(); iterator++)
	{
		#pragma region Reset_Values 
		// Reset the vectors for separation cohesion and alignment for this boid to zero
		separation.Reset(), cohesion.Reset(), alignment.Reset();
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		//sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0;

		avg_neighbour_pos.Reset(), avg_neighbour_vel.Reset(), closest_neighbour.Reset();
		distance_to_neighbour = 0.0f;
		neighbour_count = 0;
		#pragma endregion

		// For every other boid in the flock...
		for (std::vector<boid>::iterator iterator2 = boids_.begin(); iterator2 != boids_.end(); iterator2++)
		{
			// Find the shortest distance between the two boids
			float euclidean_distance = sqrtf(pow((iterator2->GetPos().x - iterator->GetPos().x), 2) + pow((iterator2->GetPos().y - iterator->GetPos().y), 2));
			// If the positions are close enough to interact
			if (euclidean_distance < interaction_distance_)
			{
				// Add the position of the neighbour to the sum of all neighbours positions
				avg_neighbour_pos += iterator2->GetPos();
				// Add the velocity of the neighbour to the sum of all neighbours positions
				avg_neighbour_vel += iterator2->GetVel();
				// Increase the number of neighbours by 1
				neighbour_count++;

				// Check that we are not calculating against itself
				if (iterator != iterator2)
				{
					if ( closest_neighbour.Length() < ( iterator2->GetPos().Length() - iterator->GetPos().Length() ) )
					{

					}
				}
			}
		}

		cohesion = Cohesion(avg_neighbour_pos, iterator->GetPos(), neighbour_count);
		alignment = Alignment(avg_neighbour_vel, neighbour_count);

	}
}

gef::Vector2 flock::Cohesion(gef::Vector2 avg_neighbour_pos, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	// Calculate the average(mean) of the positions:
	avg_neighbour_pos /= (float)neighbour_count;
	// Find the vector to the local flock centre:
	gef::Vector2 lfc_vec = avg_neighbour_pos - boid_pos;
	// Calculate the cohesion unit vector:
	result = lfc_vec / lfc_vec.Length();
	return result;
}

gef::Vector2 flock::Alignment(gef::Vector2 avg_neighbour_vel, int neighbour_count)
{
	gef::Vector2 result;
	// Calculate the average(mean) of the velocities:
	avg_neighbour_vel /= (float)neighbour_count;
	// Calculate the alignment unit vector:
	result = avg_neighbour_vel / avg_neighbour_vel.Length();
	return result;
}

void flock::PhysicsCalculations(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time)
{
	// This calculates the updated position using semi-implicit Euler
	iterator_->SetAccel(accel);

	//iterator_->WrapAround(bound_x_, bound_y_);
	//iterator_->Bounds(bound_x_, bound_y_);

	// Questions:
	// What is a reasonable velocity in this framework?
	// What does a velocity of 1 look like?
	// How does this compare to the values we get for the acceleration?
	// How can we fix this via the use of damping and weights?


	// Semi-Impicit Euler Method for updated position calculations:
	// v = u + at
	gef::Vector2 velocity = (iterator_->GetVel()) + (accel * frame_time);
	//if (velocity.Length() > max_speed_) 
	//	velocity.Limit(max_speed_);
	
	// x1 = x0 + vt
	gef::Vector2 new_pos = iterator_->GetPos() + (velocity * frame_time);

	// Update the transform of the object within the scene:
	iterator_->SetVel(velocity);
	iterator_->SetPos(new_pos);
	iterator_->GetCube()->set_transform(iterator_->GetTranslationMatrix());
}

gef::Vector2 flock::AvoidBoundary(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time)
{
	gef::Vector2 result = accel;

	if (iterator_->GetPos().x > (bound_x_ - 5.0f))
	{
		result.x = accel.x - (1.0f / (bound_x_ - iterator_->GetPos().x));
	}
	else if (iterator_->GetPos().x < -(bound_x_ - 5.0f))
	{
		result.x = accel.x + (1.0f / (bound_x_ - iterator_->GetPos().x));
	}

	if (iterator_->GetPos().y >(bound_y_ - 3.0f))
	{
		result.y = accel.y - (1.0f / (bound_y_ - iterator_->GetPos().y));
	}
	else if (iterator_->GetPos().y < -(bound_y_ - 3.0f))
	{
		result.y = accel.y + (1.0f / (bound_y_ - iterator_->GetPos().y));
	}

	return result;
}



void flock::CleanUp()
{
	for (std::vector<boid>::iterator iterator_ = boids_.begin(); iterator_ != boids_.end(); iterator_++)
	{
		iterator_->CleanUp();
	}
}
