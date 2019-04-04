#include "flock.h"

float flock::max_speed_ = 0.5f;
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

		

		boid boid_(platform_);
		boid_.Initialise();

		// Slightly randomise positions: (Surprisingly tiny variations away from a result of 5.0f create a large difference)
		float rand_jiggle = (float)(rand() % 100 + 4950) / 1000.0f;

		#pragma region boid_placement
		float x = ((float)i / rand_jiggle) * 3.14159f;
		float y = ((float)i / rand_jiggle) * 3.14159f;

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
		//separation_ *= sep_wgt_, cohesion_ *= coh_wgt_, alignment_ *= ali_wgt_;

		// Add the accelerative forces together
		gef::Vector2 acceleration = separation_ + cohesion_ + alignment_;

		// Update Physics
		PhysicsCalculations(iterator_, acceleration, frame_time);
	}
}

void flock::RunBoidsAlgorithm2(float frame_time)
{
	// Reynolds Forces
	gef::Vector2 separation, cohesion, alignment;
	// Reynolds Weights
	float coh_wgt, ali_wgt, sep_wgt;
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
				// Check that we are not calculating against itself
				if ((iterator->GetPos() - iterator2->GetPos()).Length() != 0.0f)
				{
					// Add the position of the neighbour to the sum of all neighbours positions
					avg_neighbour_pos += iterator2->GetPos();
					// Add the velocity of the neighbour to the sum of all neighbours positions
					avg_neighbour_vel += iterator2->GetVel();
					// Increase the number of neighbours by 1
					neighbour_count++;
				
					// Find the vector to the nearest flock member
					if ( closest_neighbour.Length() < ( iterator2->GetPos().Length() - iterator->GetPos().Length() ) )
					{
						closest_neighbour = iterator2->GetPos();
					}
				}
			}
		}

		if (neighbour_count != 0)
		{ 
			// Calculate the average(mean) of the positions:
			avg_neighbour_pos /= (float)neighbour_count;
			// Calculate the average(mean) of the velocities:
			avg_neighbour_vel /= (float)neighbour_count;
		}
		
		// Calculate the basic boids forces:
		cohesion   =   Cohesion(avg_neighbour_pos,                    iterator->GetPos(), neighbour_count);
		alignment  =  Alignment(avg_neighbour_pos, avg_neighbour_vel, iterator->GetPos(), neighbour_count);
		separation = Separation(closest_neighbour,                    iterator->GetPos(), neighbour_count);

		// Calculate the final force vector on the boid:
		gef::Vector2 acceleration = separation + cohesion + alignment;

		// Update Physics
		PhysicsCalculations(iterator, acceleration, frame_time);
	}
}

gef::Vector2 flock::Cohesion(gef::Vector2 avg_neighbour_pos, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Find the vector to the local flock centre:
	if ((avg_neighbour_pos - boid_pos).Length() != 0)
	{
		result = gef::Vector2(0.0f, 0.0f);
		return result;
	}
	gef::Vector2 lfc_vec = boid_pos - avg_neighbour_pos;
	// Calculate the appropriate weighting:
	weight = (boid_pos.Length() - avg_neighbour_pos.Length() ) * ( 1.0f / (30.0f * (neighbour_count * neighbour_count)) );
	// Calculate the cohesion vector active for this boid:
	result = (lfc_vec / lfc_vec.Length()) * weight;

	// Error Check:
	if (result.x != result.x || result.y != result.y)
	{
		int hello_there = 1;
	}
	return result;
}

gef::Vector2 flock::Alignment(gef::Vector2 avg_neighbour_pos, gef::Vector2 avg_neighbour_vel, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Calculate the appropriate weighting:
	weight = 1.0f / (10.0f * ((avg_neighbour_pos - boid_pos).Length()));
	// Calculate the alignment vector active for this boid:
	if (avg_neighbour_vel.Length() != 0.0f)
	{
		result = (avg_neighbour_vel / avg_neighbour_vel.Length()) * weight;
	}
	else
	{
		result = gef::Vector2(0.0f, 0.0f);
	}

	// Error Check:
	if (result.x != result.x || result.y != result.y)
	{
		int hello_there = 1;
	}

	return result;
}

gef::Vector2 flock::Separation(gef::Vector2 closest_neighbour, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight = 0.0f;

	// Calculate the vector to the nearest schoolmate
	gef::Vector2 nearest_neighbour_vec = closest_neighbour - boid_pos;
	// Calculate the appropriate weighting:
	if (neighbour_count != 0 && closest_neighbour.Length() != 0)
	{
		weight = pow(((float)neighbour_count / closest_neighbour.Length()), 2);
	}
	// Calculate the separation vector active for this boid:
	result = ( nearest_neighbour_vec / (-nearest_neighbour_vec.Length()) ) * weight;

	// Error Check:
	if (result.x != result.x || result.y != result.y)
	{
		int hello_there = 1;
	}

	return result;
}

void flock::PhysicsCalculations(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time)
{
	//iterator_->WrapAround(bound_x_, bound_y_);
	iterator_->Bounds(bound_x_, bound_y_);

	gef::Vector2 drag = iterator_->GetVel();

	// Semi-Impicit Euler Method for updated position calculations: (With Drag)
	// v = u + at
	gef::Vector2 velocity = iterator_->GetVel() + (accel * frame_time);

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
