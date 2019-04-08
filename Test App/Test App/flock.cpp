#include "flock.h"

float flock::max_speed_ = 1.0f;
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
	RunBoidsAlgorithm(frame_time);

	// Run the Genetic Algorithm


}

void flock::RunBoidsAlgorithm(float frame_time)
{
	// Reynolds Forces
	gef::Vector2 separation, cohesion, alignment;
	// Neighbour variables for boids calculations:
	gef::Vector2 avg_neighbour_pos(0.0f, 0.0f);
	gef::Vector2 avg_neighbour_vel(0.0f, 0.0f);
	gef::Vector2 closest_neighbour(0.0f, 0.0f);
	float distance_to_closest_neighbour;
	int neighbour_count;
	// Unit vector for collisions:
	gef::Vector2 collision_vector;

	for (std::vector<boid>::iterator iterator = boids_.begin(); iterator != boids_.end(); iterator++)
	{
		#pragma region Reset_Values 
		avg_neighbour_pos.Reset(), avg_neighbour_vel.Reset(), closest_neighbour.Reset(), collision_vector.Reset();
		// Arbitrarily large unobtainable number:
		distance_to_closest_neighbour = 10000.0f;
		neighbour_count = 0;
		#pragma endregion

		// For every other boid in the flock...
		for (std::vector<boid>::iterator iterator2 = boids_.begin(); iterator2 != boids_.end(); iterator2++)
		{
			// Identify the distance between the two boids:
			gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
			float shortest_distance = distance_vector.Length();

			// If the positions are close enough to interact
			if (shortest_distance < interaction_distance_)
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
					if (distance_to_closest_neighbour > shortest_distance)
					{
						closest_neighbour = iterator2->GetPos();
						distance_to_closest_neighbour = shortest_distance;
					}

					// Produces a unit vector with force inversely proportional to the distance away from its neighbour:
					if (CollisionDetection(iterator->GetMesh()->bounding_sphere().radius(), shortest_distance))
					{
						collision_vector += ( ((distance_vector * 2.5f) / shortest_distance) / shortest_distance);
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

			// Calculate the basic boids forces:
			cohesion = Cohesion(avg_neighbour_pos, iterator->GetPos(), neighbour_count);
			alignment = Alignment(avg_neighbour_pos, avg_neighbour_vel, iterator->GetPos(), neighbour_count);
			separation = Separation(closest_neighbour, iterator->GetPos(), neighbour_count);
		}
		else
		{
			// There is no flocking behaviour:
			cohesion.Reset(), alignment.Reset(), separation.Reset();
		}
		
		// Calculate the final force vector on the boid:
		gef::Vector2 acceleration = separation + cohesion + alignment - collision_vector;
		// Update Physics
		PhysicsCalculations(iterator, acceleration, frame_time);
	}
}

gef::Vector2 flock::Cohesion(gef::Vector2 avg_neighbour_pos, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Find the vector to the local flock centre:
	if ((avg_neighbour_pos - boid_pos).Length() == 0)
	{
		result = gef::Vector2(0.0f, 0.0f);
		return result;
	}
	gef::Vector2 lfc_vec = boid_pos - avg_neighbour_pos;
	// Calculate the appropriate weighting:
	weight = 20.0f * (boid_pos.Length() - avg_neighbour_pos.Length()) / (1.0f * (neighbour_count * neighbour_count));
	// Calculate the cohesion vector active for this boid:
	result = (lfc_vec / lfc_vec.Length()) * weight;

	// Error Check:
	if (result.x != result.x || result.y != result.y)
	{
		int hello_there = 1;
		result = gef::Vector2(0.0f, 0.0f);
		return result;
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
		result = gef::Vector2(0.0f, 0.0f);
		return result;
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
		weight = 0.25 * pow(((float)neighbour_count / closest_neighbour.Length()), 2);
	}
	// Calculate the separation vector active for this boid:
	result = ( nearest_neighbour_vec / (-nearest_neighbour_vec.Length()) ) * weight;

	// Error Check:
	if (result.x != result.x || result.y != result.y)
	{
		int hello_there = 1;
		result = gef::Vector2(0.0f, 0.0f);
		return result;
	}

	return result;
}

bool flock::CollisionDetection(float combined_radii_length, float shortest_distance)
{
	//float combined_radii_length = iterator->GetMesh()->bounding_sphere().radius() + iterator2->GetMesh()->bounding_sphere().radius();

	if (shortest_distance < (combined_radii_length*2.0f))
	{
		return true;
	}

	return false;
}

void flock::PhysicsCalculations(std::vector<boid>::iterator iterator_, gef::Vector2 accel, float frame_time)
{
	//iterator_->WrapAround(bound_x_, bound_y_);
	iterator_->Bounds(bound_x_, bound_y_);

	gef::Vector2 drag = iterator_->GetVel();

	// Semi-Impicit Euler Method for updated position calculations: (With Drag)
	// v = u + at
	gef::Vector2 velocity = iterator_->GetVel() + (accel * frame_time);
	velocity.Limit(max_speed_);

	// x1 = x0 + vt
	gef::Vector2 new_pos = iterator_->GetPos() + (velocity * frame_time);

	// Update the transform of the object within the scene:
	iterator_->SetVel(velocity);
	iterator_->SetPos(new_pos);
	iterator_->GetMeshInstance()->set_transform(iterator_->GetTranslationMatrix());
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
