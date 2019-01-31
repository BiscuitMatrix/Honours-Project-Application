#include "boid.h"



boid::boid(gef::Platform& platform) :
	platform_(platform),
	mesh_(nullptr),
	cube_(nullptr),
	curr_vel_(0.0f, 0.0f)
{
}


boid::~boid()
{
}

void boid::Initialise()
{
	mesh_ = new gef::Mesh(platform_);
	mesh_ = CreateCubeMesh();

	cube_ = new gef::MeshInstance();
	cube_->set_mesh(mesh_);

	// Reynolds Forces
	separation_ = &gef::Vector2(0.0f, 0.0f);
	cohesion_ = &gef::Vector2(0.0f, 0.0f);
	alignment_ = &gef::Vector2(0.0f, 0.0f);

	// Desired amount of separation
	desired_separation_ = 1.5f;

	// Reynolds Weights
	sep_wgt_ = 1.5f, coh_wgt_ = 1.5f, ali_wgt_ = 1.5f;
	// Reynolds Counters
	sep_counter_, ali_counter_, coh_counter_ = 0;
	// Limits
	max_speed_ = 5.0f;
	max_force_ = 1.0f;

	scale_.SetIdentity();
	rotation_.SetIdentity();
	translation_.SetIdentity();
}

void boid::Update(float frame_time)
{
	UpdatePosition(frame_time);
}

void boid::RunBoidsAlgorithm(std::vector<boid>* boid, float frame_time)
{
	for (iterator_ = boid->begin(); iterator_ != boid->end(); iterator_++)
	{
		// Set the vectors for separation cohesion and alignment for this boid to zero
		iterator_->separation_->x = 0.0f;
		iterator_->separation_->y = 0.0f;
		iterator_->cohesion_->x = 0.0f;
		iterator_->cohesion_->y = 0.0f;
		iterator_->alignment_->x = 0.0f;
		iterator_->alignment_->y = 0.0f;

		// Define vectors to take the sume of positions and velocities of neighbouring boids respectively
		gef::Vector2 sum_of_positions(0, 0), sum_of_velocities(0, 0);
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		sep_counter_ = 0, ali_counter_ = 0, coh_counter_ = 0;

		// For every other boid in the flock
		for (iterator_2_ = boid->begin(); iterator_2_ != boid->end(); iterator_2_++)
		{
			// 0.1.1. Check that we are not calculating against itself
			if (iterator_ != iterator_2_)
			{
				// 0. Find the Euclidean distance between the two boids
				float distance = sqrtf(pow((iterator_2_->GetCurrPos().x - iterator_->GetCurrPos().x), 2) + pow((iterator_2_->GetCurrPos().y - iterator_->GetCurrPos().y), 2));
				// 0.1. If the positions are close enough to interact
				if (distance < 50.0f)
				{
					// 1. Separation
					if (distance < iterator_->desired_separation_)
					{
						// 1.1. Take the difference in position
						gef::Vector2 difference;
						difference = (iterator_->GetCurrPos() - iterator_2_->GetCurrPos());
						// 1.2. Find the unit vector (or v hat) values in x and y 
						difference.Normalise();
						// 1.3. Weight by the distance between the two
						difference /= distance;
						// 1.4. Add this difference to the separation value
						*iterator_->separation_ += difference;
						// 1.5. Increase the counter to use for division later
						iterator_->sep_counter_++;
					}

					// 2. Cohesion
					// 2.1. Add the position of the neighbour to the sum of all neighbours positions
					*iterator_->cohesion_ += iterator_2_->GetCurrPos();
					// 2.2. increase the counter to use for division later
					iterator_->coh_counter_++;

					// 3. Alignment
					// 3.1. add the velocity of the neighbour to the sum of all neighbours velocities
					*iterator_->alignment_ += iterator_2_->GetCurrPos();
					// 3.2. increase the counter to for division later
					iterator_->ali_counter_++;
				}
			}
		}

		// 1. Separation Part 2
		if (iterator_->sep_counter_ > 1)
		{
			// Calculate the mean of the acted forces
			*iterator_->separation_ /= (float)iterator_->sep_counter_;
		}

		if (iterator_->separation_->Length() > 0.0f)
		{
			// Implement Reynolds: Steering = Desired - Velocity
			iterator_->separation_->Normalise();
			// Multiply the force by the max speed variable
			*iterator_->separation_ *= max_speed_;
			// Minus the current velocity of the boid
			*iterator_->separation_ -= iterator_->curr_vel_;
			// limit the magnitude of the sep vector
			iterator_->separation_->Limit(max_force_);
		}


		// 2. Cohesion Part 2
		if (iterator_->coh_counter_ > 1)
		{
			// Calculate the mean of the acted forces
			*iterator_->cohesion_ /= (float)iterator_->coh_counter_;
		}
		else
		{
			*iterator_->cohesion_ = gef::Vector2(0.0f, 0.0f);
		}

		// 3. Alignment Part 2
		if (iterator_->ali_counter_ > 0)
		{
			if (iterator_->ali_counter_ > 1)
			{
				// Calculate the mean of the acted forces
				*iterator_->alignment_ /= (float)iterator_->ali_counter_;
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

		// Add the accelerativew forces together
		iterator_->accel_ = *iterator_->separation_ + *iterator_->cohesion_ + *iterator_->alignment_;

		// v = u + at
		iterator_->curr_vel_ = (iterator_->prev_vel_) + (iterator_->accel_ * frame_time);

		// s = ut + 1/2(a(t^2))
		iterator_->displacement_ = (iterator_->prev_vel_ * frame_time) + ((iterator_->accel_*pow(frame_time, 2)) / 2.0f);

		iterator_->curr_pos_ += iterator_->displacement_;

		iterator_->WrapAround(30.0f, 30.0f);
		
		// Set the boids prev values so we can reference the values of the previous frame in the next frame
		iterator_->prev_vel_ = iterator_->curr_vel_;
		iterator_->prev_pos_ = iterator_->curr_pos_;

		gef::Vector4 pos = gef::Vector4(iterator_->curr_pos_.x, 0.0f, iterator_->curr_pos_.y);
		iterator_->translation_.SetTranslation(pos);

		//gef::Matrix44 final_transform = iterator_->scale_ * iterator_->rotation_ * iterator_->translation_;
		gef::Matrix44 final_transform = iterator_->translation_;
		iterator_->cube_->set_transform(final_transform);
	}
}

void boid::CollDetect()
{
}

void boid::Bounds(float x, float z)
{
	if (curr_pos_.x > x) curr_pos_.x = x;
	if (curr_pos_.x < -x) curr_pos_.x = -x;
	if (curr_pos_.y > z) curr_pos_.y = z;
	if (curr_pos_.y < -z) curr_pos_.y = -z;
}

void boid::WrapAround(float x, float z)
{
	if (curr_pos_.x < -x) curr_pos_.x = x;
	if (curr_pos_.y < -z) curr_pos_.y = z;
	if (curr_pos_.x > x) curr_pos_.x = -x;
	if (curr_pos_.y > z) curr_pos_.y = -z;
}


void boid::UpdatePosition(float frame_time)
{
	// Add the accelerativew forces together
	accel_ = *separation_;// -*cohesion_ - *alignment_;
	// s = ut + 1/2(a(t^2))
	displacement_ = (prev_pos_ * frame_time) + ((accel_*pow(frame_time, 2)) / 2);

	curr_pos_ += displacement_;

	prev_pos_ = curr_pos_;

	gef::Vector4 pos = gef::Vector4(curr_pos_.x, 0.0f, curr_pos_.y);
	translation_.SetTranslation(pos);

	gef::Matrix44 final_transform = scale_ * rotation_ * translation_;
	cube_->set_transform(final_transform);
}

void boid::CleanUp()
{
	delete mesh_;
	mesh_ = nullptr;
}

gef::Mesh* boid::CreateCubeMesh()
{
	gef::Mesh* mesh = new gef::Mesh(platform_);

	// initialise the vertex data to create a 1, 1, 1 cube
	const float half_size = 0.5f;
	const gef::Mesh::Vertex vertices[] = {
		{ half_size, -half_size, -half_size,  0.577f, -0.577f, -0.577f, 0.0f, 0.0f },
	{ half_size,  half_size, -half_size,  0.577f,  0.577f, -0.577f, 0.0f, 0.0f },
	{ -half_size,  half_size, -half_size, -0.577f,  0.577f, -0.577f, 0.0f, 0.0f },
	{ -half_size, -half_size, -half_size, -0.577f, -0.577f, -0.577f, 0.0f, 0.0f },
	{ half_size, -half_size,  half_size,  0.577f, -0.577f,  0.577f, 0.0f, 0.0f },
	{ half_size,  half_size,  half_size,  0.577f,  0.577f,  0.577f, 0.0f, 0.0f },
	{ -half_size,  half_size,  half_size, -0.577f,  0.577f,  0.577f, 0.0f, 0.0f },
	{ -half_size, -half_size,  half_size, -0.577f, -0.577f,  0.577f, 0.0f, 0.0f }
	};

	mesh->InitVertexBuffer(platform_, static_cast<const void*>(vertices), sizeof(vertices) / sizeof(gef::Mesh::Vertex), sizeof(gef::Mesh::Vertex));

	// we will create a single triangle list primitive to draw the triangles that make up the cube
	mesh->AllocatePrimitives(1);
	gef::Primitive* primitive = mesh->GetPrimitive(0);

	const UInt32 indices[] = {
		// Back
		0, 1, 2,
		2, 3, 0,
		// Front
		6, 5, 4,
		4, 7, 6,
		// Left
		2, 7, 3,
		2, 6, 7,
		// Right
		0, 4, 1,
		5, 1, 4,
		// Top
		6, 2, 1,
		5, 6, 1,
		// Bottom
		0, 3, 7,
		0, 7, 4
	};

	primitive->InitIndexBuffer(platform_, static_cast<const void*>(indices), sizeof(indices) / sizeof(UInt32), sizeof(UInt32));
	primitive->set_type(gef::TRIANGLE_LIST);

	// set size of bounds, we need this for collision detection routines
	gef::Aabb aabb(gef::Vector4(-half_size, -half_size, -half_size), gef::Vector4(half_size, half_size, half_size));
	gef::Sphere sphere(aabb);

	mesh->set_aabb(aabb);
	mesh->set_bounding_sphere(sphere);

	return mesh;
}
