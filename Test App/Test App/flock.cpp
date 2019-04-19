#include "flock.h"

float flock::max_speed_ = 1.0f;
float flock::max_force_ = 3.0f;

float flock::bound_x_ = 55.0f;
float flock::bound_y_ = 30.0f;

float flock::food_detection_sqr_ = 225.0f;
float flock::interaction_distance_sqr_ = 100.0f;

flock::flock(gef::Platform& platform, bool isGAenabled) :
	platform_(platform),
	GA_enabled_(isGAenabled)
{
}

flock::~flock()
{
}


void flock::Initialise(gef::Vector2 flock_centre)
{
	for (int i = 0; i < glo_flock_size; i++)
	{
		boid boid_(platform_);
		boid_.Initialise();

		// Slightly randomise positions: (Surprisingly tiny variations away from a result of 5.0f create a large positional difference)
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

	if (GA_enabled_)
	{
		// Genetic Algorithm Settings
		genetic_algorithm_ = new genetic_algorithm();
		genetic_algorithm_->Initialise(&boids_);
	}
}

void flock::Update(std::vector<boid> *enemy_boids, std::vector<resource> *resources, float frame_time)
{
	// Take copies of the vector lists of the other entities within the program:
	enemy_boids_ = enemy_boids;
	resources_ = resources;

	// Run boids algorithm
	RunBoidsAlgorithm(frame_time);

	// Run the Genetic Algorithm


}

void flock::RunBoidsAlgorithm(float frame_time)
{
	// Reynolds Forces
	gef::Vector2 separation, cohesion, alignment;
	// Expanded Forces
	gef::Vector2 food_attraction, flock_avoidance, free_movement, boundary;
	// Neighbour variables for boids calculations:
	gef::Vector2 avg_neighbour_pos(0.0f, 0.0f);
	gef::Vector2 avg_neighbour_vel(0.0f, 0.0f);
	gef::Vector2 closest_neighbour(0.0f, 0.0f);
	float distance_to_closest_neighbour_sqr;
	int neighbour_count;
	// Unit vector for collisions:
	gef::Vector2 collision_vector;

	for (std::vector<boid>::iterator iterator = boids_.begin(); iterator != boids_.end(); iterator++)
	{
		#pragma region Reynolds Boids
		avg_neighbour_pos.Reset(), avg_neighbour_vel.Reset(), closest_neighbour.Reset(), collision_vector.Reset();
		// Arbitrarily large unobtainable number:
		distance_to_closest_neighbour_sqr = 1000000.0f;
		neighbour_count = 0;

		// For every other boid in the flock...
		for (std::vector<boid>::iterator iterator2 = boids_.begin(); iterator2 != boids_.end(); iterator2++)
		{
			// Identify the distance between the two boids:
			gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
			float shortest_distance_sqr = distance_vector.LengthSqr();

			// If the positions are close enough to interact
			if (shortest_distance_sqr < interaction_distance_sqr_)
			{
				// Check that we are not calculating against itself
				if ((iterator->GetPos() - iterator2->GetPos()).LengthSqr() != 0.0f)
				{
					// Add the position of the neighbour to the sum of all neighbours positions
					avg_neighbour_pos += iterator2->GetPos();
					// Add the velocity of the neighbour to the sum of all neighbours velocities
					avg_neighbour_vel += iterator2->GetVel();
					// Increase the number of neighbours by 1
					neighbour_count++;
				
					// Find the vector to the nearest flock member
					if (distance_to_closest_neighbour_sqr > shortest_distance_sqr)
					{
						closest_neighbour = iterator2->GetPos();
						distance_to_closest_neighbour_sqr = shortest_distance_sqr;
					}

					// Produces a unit vector with force inversely proportional to the distance away from its neighbour:
					if (CollisionDetection(iterator->GetMesh()->bounding_sphere().radius(), shortest_distance_sqr))
					{
						collision_vector += ( ((distance_vector * 5.0f) / shortest_distance_sqr) / shortest_distance_sqr);
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

			if (GA_enabled_)
			{
				// Calculate the basic boids forces:
				cohesion = GACohesion(iterator->GetDNA(), avg_neighbour_pos, iterator->GetPos(), neighbour_count);
				alignment = GAAlignment(iterator->GetDNA(), avg_neighbour_pos, avg_neighbour_vel, iterator->GetPos(), neighbour_count);
				separation = GASeparation(iterator->GetDNA(), closest_neighbour, iterator->GetPos(), neighbour_count);
			}
			else
			{
				// Calculate the basic boids forces:
				cohesion = Cohesion(avg_neighbour_pos, iterator->GetPos(), neighbour_count);
				alignment = Alignment(avg_neighbour_pos, avg_neighbour_vel, iterator->GetPos(), neighbour_count);
				separation = Separation(closest_neighbour, iterator->GetPos(), neighbour_count);
			}
		}
		else
		{
			// There is no flocking behaviour:
			cohesion.Reset(), alignment.Reset(), separation.Reset();
		}
		#pragma endregion

		if (GA_enabled_)
		{
			food_attraction = GAFoodAttraction(iterator);
			flock_avoidance = GAFlockAvoidance(iterator);
		}
		else
		{
			food_attraction = FoodAttraction(iterator);
			flock_avoidance = FlockAvoidance(iterator);
		}
		
		// Calculate the final force vector on the boid:
		gef::Vector2 acceleration = separation + cohesion + alignment - collision_vector + food_attraction + flock_avoidance;
		// Update Physics
		PhysicsCalculations(iterator, acceleration, frame_time);

		// Update boid specific calculations
		iterator->Update(frame_time);
	}
}


gef::Vector2 flock::Cohesion(gef::Vector2 avg_neighbour_pos, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Find the vector to the local flock centre:
	if ((avg_neighbour_pos - boid_pos).LengthSqr() == 0)
	{
		result = gef::Vector2(0.0f, 0.0f);
		return result;
	}
	gef::Vector2 lfc_vec = boid_pos - avg_neighbour_pos;
	// Calculate the appropriate weighting:
	weight = (pow(boid_pos.Length() - avg_neighbour_pos.Length(), 2)) / (30.0f * (float)glo_flock_size*(float)glo_flock_size);
	// Calculate the cohesion vector active for this boid:
	result = (lfc_vec / lfc_vec.Length()) * weight;

	return result;
}
gef::Vector2 flock::Alignment(gef::Vector2 avg_neighbour_pos, gef::Vector2 avg_neighbour_vel, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Calculate the appropriate weighting:
	weight = 1.0f / (10.0f * ((avg_neighbour_pos - boid_pos).Length()));
	// Calculate the alignment vector active for this boid:
	if (avg_neighbour_vel.LengthSqr() != 0.0f)
	{
		result = (avg_neighbour_vel / avg_neighbour_vel.Length()) * weight;
	}
	else
	{
		result = gef::Vector2(0.0f, 0.0f);
	}

	return result;
}
gef::Vector2 flock::Separation(gef::Vector2 closest_neighbour, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight = 0.0f;

	// Calculate the vector to the nearest flock member
	gef::Vector2 nearest_neighbour_vec = closest_neighbour - boid_pos;
	// Calculate the appropriate weighting:
	if (neighbour_count != 0 && closest_neighbour.LengthSqr() != 0)
	{
		weight = 0.025f * pow(((float)neighbour_count / closest_neighbour.Length()), 2);
	}
	// Calculate the separation vector active for this boid:
	result = ( nearest_neighbour_vec / (-nearest_neighbour_vec.Length()) ) * weight;

	return result;
}
gef::Vector2 flock::FoodAttraction(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result;
	float weight = 0.0f;
	// Resource variables for boids calculations:
	gef::Vector2 closest_resource(5000.0f, 5000.0f);
	gef::Vector2 original_value(5000.0f, 5000.0f);
	float distance_to_closest_resource_sqr = 1000000.0f;

	// For each resource in the environment:
	for (std::vector<resource>::iterator iterator2 = resources_->begin(); iterator2 != resources_->end(); iterator2++)
	{
		if (iterator2->GetActive())
		{
			// Identify the distance between the two boids:
			gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
			float shortest_distance_sqr = distance_vector.LengthSqr();

			// If the positions are close enough to interact
			if (shortest_distance_sqr < food_detection_sqr_)
			{
				// Find the vector to the nearest resource:
				if (distance_to_closest_resource_sqr > shortest_distance_sqr)
				{
					closest_resource = iterator2->GetPos();
					distance_to_closest_resource_sqr = shortest_distance_sqr;
				}

				if (CollisionDetection((iterator->GetMesh()->bounding_sphere().radius() - 0.3f), shortest_distance_sqr))
				{
					iterator2->SetActive(false);
					iterator->HasEaten();
				}
			}
		}
	}

	if ((closest_resource - original_value).LengthSqr() != 0)
	{
		// Calculate the vector to the nearest resource:
		gef::Vector2 nearest_resource_vec = closest_resource - iterator->GetPos();
		// Calculate the appropriate weighting:
		weight = (0.0025f * distance_to_closest_resource_sqr) + (36.0f / distance_to_closest_resource_sqr);

		// Calculate the food attraction vector active for this boid:
		result = (nearest_resource_vec / nearest_resource_vec.Length()) * weight;
	}
	else
	{
		result.Reset();
	}

	return result;
}
gef::Vector2 flock::FlockAvoidance(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result;
	float weight = 0.0f;

	gef::Vector2 avg_other_flock_pos(0.0f, 0.0f);
	float other_flock_count = 0;

	gef::Vector2 other_collision_vector(0.0f, 0.0f);

	// For each resource in the environment:
	for (std::vector<boid>::iterator iterator2 = enemy_boids_->begin(); iterator2 != enemy_boids_->end(); iterator2++)
	{
		// Identify the distance between the two boids:
		gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
		float shortest_distance_sqr = distance_vector.LengthSqr();

		// If the positions are close enough to interact
		if (shortest_distance_sqr < interaction_distance_sqr_)
		{
			// Add the position of the neighbour to the sum of all neighbours positions
			avg_other_flock_pos += iterator2->GetPos();
			// Increase the number of neighbours by 1
			other_flock_count++;

			if (CollisionDetection((iterator->GetMesh()->bounding_sphere().radius() + 1.0f), shortest_distance_sqr))
			{
				other_collision_vector += (((distance_vector * 2.5f) / shortest_distance_sqr) / shortest_distance_sqr);
			}
		}
	}

	if (other_flock_count != 0)
	{
		// Calculate the average(mean) of the positions:
		avg_other_flock_pos /= (float)other_flock_count;

		// Calculate the force applied by the other flock:
		gef::Vector2 other_flock_vec = avg_other_flock_pos - iterator->GetPos();
		// Calculate the appropriate weighting:
		weight = 300.0f / (avg_other_flock_pos.Length());

		// Calculate the repelling force vector active for this boid from the other flock:
		result = ((other_flock_vec / -other_flock_vec.Length()) * weight);// -other_collision_vector;
	}
	else
	{
		// There is no flocking behaviour:
		result.Reset();
	}

	return result;
}
gef::Vector2 flock::Boundary(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result; 
	float weight;
	gef::Vector2 c1(bound_x_, bound_y_);
	gef::Vector2 c2(-bound_x_, -bound_y_);
	gef::Vector2 nearest_boundary_vec;

	// Calculate the vector to the nearest border:
	if ((c1 - iterator->GetPos()).LengthSqr() < (c2 - iterator->GetPos()).LengthSqr())
	{
		nearest_boundary_vec = c1 - iterator->GetPos();
		//weight = pow(((float)neighbour_count / closest_neighbour.Length()), 2);
	}
	else
	{
		nearest_boundary_vec = c2 - iterator->GetPos();
	}
	


	return result;
}


gef::Vector2 flock::GACohesion(DNA dna, gef::Vector2 avg_neighbour_pos, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Find the vector to the local flock centre:
	if ((avg_neighbour_pos - boid_pos).LengthSqr() == 0)
	{
		result = gef::Vector2(0.0f, 0.0f);
		return result;
	}
	gef::Vector2 lfc_vec = boid_pos - avg_neighbour_pos;
	// Calculate the appropriate weighting:
	weight = (dna.GetData(0) * pow(boid_pos.Length() - avg_neighbour_pos.Length(), 2)) / (dna.GetData(1) * (float)glo_flock_size*(float)glo_flock_size);
	// Calculate the cohesion vector active for this boid:
	result = (lfc_vec / lfc_vec.Length()) * weight;

	return result;
}
gef::Vector2 flock::GAAlignment(DNA dna, gef::Vector2 avg_neighbour_pos, gef::Vector2 avg_neighbour_vel, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight;

	// Calculate the appropriate weighting:
	weight = dna.GetData(2) / (dna.GetData(3) * ((avg_neighbour_pos - boid_pos).Length()));
	// Calculate the alignment vector active for this boid:
	if (avg_neighbour_vel.LengthSqr() != 0.0f)
	{
		result = (avg_neighbour_vel / avg_neighbour_vel.Length()) * weight;
	}
	else
	{
		result = gef::Vector2(0.0f, 0.0f);
	}

	return result;
}
gef::Vector2 flock::GASeparation(DNA dna, gef::Vector2 closest_neighbour, gef::Vector2 boid_pos, int neighbour_count)
{
	gef::Vector2 result;
	float weight = 0.0f;

	// Calculate the vector to the nearest flock member
	gef::Vector2 nearest_neighbour_vec = closest_neighbour - boid_pos;
	// Calculate the appropriate weighting:
	if (neighbour_count != 0 && closest_neighbour.LengthSqr() != 0)
	{
		weight = dna.GetData(4) * pow(((dna.GetData(5) * (float)neighbour_count) / (dna.GetData(6) * closest_neighbour.Length())), 2);
	}
	// Calculate the separation vector active for this boid:
	result = (nearest_neighbour_vec / (-nearest_neighbour_vec.Length())) * weight;

	return result;
}
gef::Vector2 flock::GAFoodAttraction(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result;
	float weight = 0.0f;
	// Resource variables for boids calculations:
	gef::Vector2 closest_resource(5000.0f, 5000.0f);
	gef::Vector2 original_value(5000.0f, 5000.0f);
	float distance_to_closest_resource_sqr = 1000000.0f;

	// For each resource in the environment:
	for (std::vector<resource>::iterator iterator2 = resources_->begin(); iterator2 != resources_->end(); iterator2++)
	{
		if (iterator2->GetActive())
		{
			// Identify the distance between the two boids:
			gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
			float shortest_distance_sqr = distance_vector.LengthSqr();

			// If the positions are close enough to interact
			if (shortest_distance_sqr < food_detection_sqr_)
			{
				// Find the vector to the nearest resource:
				if (distance_to_closest_resource_sqr > shortest_distance_sqr)
				{
					closest_resource = iterator2->GetPos();
					distance_to_closest_resource_sqr = shortest_distance_sqr;
				}

				if (CollisionDetection((iterator->GetMesh()->bounding_sphere().radius() - 0.3f), shortest_distance_sqr))
				{
					iterator2->SetActive(false);
					iterator->HasEaten();
				}
			}
		}
	}

	if ((closest_resource - original_value).LengthSqr() != 0)
	{
		// Calculate the vector to the nearest resource:
		gef::Vector2 nearest_resource_vec = closest_resource - iterator->GetPos();
		// Calculate the appropriate weighting:
		weight = (iterator->GetDNA().GetData(7) * distance_to_closest_resource_sqr) + (iterator->GetDNA().GetData(8) / (iterator->GetDNA().GetData(9) * distance_to_closest_resource_sqr));

		// Calculate the food attraction vector active for this boid:
		result = (nearest_resource_vec / nearest_resource_vec.Length()) * weight;
	}
	else
	{
		result.Reset();
	}

	return result;
}
gef::Vector2 flock::GAFlockAvoidance(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result;
	float weight = 0.0f;

	gef::Vector2 avg_other_flock_pos(0.0f, 0.0f);
	float other_flock_count = 0;

	gef::Vector2 other_collision_vector(0.0f, 0.0f);

	// For each resource in the environment:
	for (std::vector<boid>::iterator iterator2 = enemy_boids_->begin(); iterator2 != enemy_boids_->end(); iterator2++)
	{
		// Identify the distance between the two boids:
		gef::Vector2 distance_vector = gef::Vector2(iterator2->GetPos().x - iterator->GetPos().x, iterator2->GetPos().y - iterator->GetPos().y);
		float shortest_distance_sqr = distance_vector.LengthSqr();

		// If the positions are close enough to interact
		if (shortest_distance_sqr < interaction_distance_sqr_)
		{
			// Add the position of the neighbour to the sum of all neighbours positions
			avg_other_flock_pos += iterator2->GetPos();
			// Increase the number of neighbours by 1
			other_flock_count++;

			if (CollisionDetection((iterator->GetMesh()->bounding_sphere().radius() + 1.0f), shortest_distance_sqr))
			{
				other_collision_vector += (((distance_vector * 2.5f) / shortest_distance_sqr) / shortest_distance_sqr);
			}
		}
	}

	if (other_flock_count != 0)
	{
		// Calculate the average(mean) of the positions:
		avg_other_flock_pos /= (float)other_flock_count;

		// Calculate the force applied by the other flock:
		gef::Vector2 other_flock_vec = avg_other_flock_pos - iterator->GetPos();
		// Calculate the appropriate weighting:
		weight = iterator->GetDNA().GetData(10) / (iterator->GetDNA().GetData(11) * avg_other_flock_pos.Length());

		// Calculate the repelling force vector active for this boid from the other flock:
		result = ((other_flock_vec / -other_flock_vec.Length()) * weight);// -other_collision_vector;
	}
	else
	{
		// There is no flocking behaviour:
		result.Reset();
	}

	return result;
}
gef::Vector2 flock::GABoundary(std::vector<boid>::iterator iterator)
{
	gef::Vector2 result;
	float weight;
	gef::Vector2 c1(bound_x_, bound_y_);
	gef::Vector2 c2(-bound_x_, -bound_y_);
	gef::Vector2 nearest_boundary_vec;

	// Calculate the vector to the nearest border:
	if ((c1 - iterator->GetPos()).LengthSqr() < (c2 - iterator->GetPos()).LengthSqr())
	{
		nearest_boundary_vec = c1 - iterator->GetPos();
		//weight = pow(((float)neighbour_count / closest_neighbour.Length()), 2);
	}
	else
	{
		nearest_boundary_vec = c2 - iterator->GetPos();
	}



	return result;
}


bool flock::CollisionDetection(float combined_radii_length, float shortest_distance_sqr)
{
	if (shortest_distance_sqr < ((combined_radii_length + combined_radii_length)*(combined_radii_length + combined_radii_length)))
	{
		return true;
	}
	return false;
}
void flock::PhysicsCalculations(std::vector<boid>::iterator iterator, gef::Vector2 accel, float frame_time)
{
	//iterator_->WrapAround(bound_x_, bound_y_);
	iterator->Bounds(bound_x_, bound_y_);

	// Semi-Impicit Euler Method for updated position calculations: 
	// v = u + at
	gef::Vector2 velocity = iterator->GetVel() + (accel * frame_time);
	velocity.Limit(max_speed_);
	gef::Vector2 drag = (velocity / -velocity.Length()) * (velocity.Length() / 2.0f);

	// x1 = x0 + vt
	gef::Vector2 new_pos = iterator->GetPos() + ((velocity - drag) * frame_time);

	// Update the transform of the object within the scene:
	iterator->SetVel(velocity);
	iterator->SetPos(new_pos);
	iterator->GetMeshInstance()->set_transform(iterator->GetTranslationMatrix());
}


void flock::CleanUp()
{
	for (std::vector<boid>::iterator iterator_ = boids_.begin(); iterator_ != boids_.end(); iterator_++)
	{
		iterator_->CleanUp();
	}
}
