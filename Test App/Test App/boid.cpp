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

	desired_separation_ = 0.05f;
	// Reynolds Weights
	sep_wgt_, coh_wgt_, ali_wgt = 1.5f;
	// Reynolds Counters
	sep_counter_, ali_counter_, coh_counter_ = 0;

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
		// Define the vectors for separation, cohesion and alignment
		gef::Vector2  sep(0.0f, 0.0f), coh(0.0f, 0.0f), ali(0.0f, 0.0f);

		// Define vectors to take the sume of positions and velocities of neighbouring boids respectively
		gef::Vector2 sum_of_positions(0, 0), sum_of_velocities(0, 0);
		// Define counters to use to take the mean values of each of the primary vectors (sep, coh and ali)
		sep_counter_, ali_counter_, coh_counter_ = 0;

		// For every other boid in the flock
		for (iterator_2_ = boid->begin(); iterator_2_ != boid->end(); iterator_2_++)
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
						sep_counter_++;
					}

					// 2. Cohesion
					// 2.1. Add the position of the neighbour to the sum of all neighbours positions
					coh += iterator_2_->GetCurrPos();
					// 2.2. increase the counter to use for division later
					coh_counter_++;

					// 3. Alignment
					// 3.1. add the velocity of the neighbour to the sum of all neighbours velocities
					ali += iterator_2_->GetCurrPos();
					// 3.2. increase the counter to for division later
					ali_counter_++;
				}
			}
		}

		// 1. Separation Part 2
		if (sep_counter_ > 0)
		{
			// Calculate the mean of the acted forces
			sep /= (float)sep_counter_;
		}

		if (sep.Length() < 0.0f)
		{
			// 1.1 Set the magnitude of the vector to "maxspeed"

			// 1.2. Implement Reynolds: Steering = Desired - Velocity
			sep.Normalise();
			//sep *= maxspeed;
			//sep -= velocity;
			// limit the magnitude of the sep vector -> sep.limit(maxforce);
		}


		// 2. Cohesion Part 2
		if (coh_counter_ > 0)
		{
			// Calculate the mean of the acted forces
			coh /= (float)coh_counter_;
		}
		else
		{
			coh = gef::Vector2(0.0f, 0.0f);
		}

		// 3. Alignment Part 2
		if (ali_counter_ > 0)
		{
			// Calculate the mean of the acted forces
			ali /= (float)ali_counter_;
#
			// 1.1 Set the magnitude of the vector to "maxspeed"

			// 1.2. Implement Reynolds: Steering = Desired - Velocity
			ali.Normalise();
			//ali *= maxspeed;
			//ali -= velocity;
			// limit the magnitude of the sep vector -> ali.limit(maxforce);
		}
		else
		{
			ali = gef::Vector2(0.0f, 0.0f);
		}

		// Weight the final value
		sep *= sep_wgt_;
		coh *= 1.5f;
		ali *= 1.5f;

		// Update the boids values with the changes
		iterator_->SetSeparationVector(&sep);
		iterator_->SetCohesionVector(&coh);
		iterator_->SetAlignmentVector(&ali);

		// Update the boid now all the information surrounding it has been updated
		//iterator_->Update(frame_time);
	}
}

void boid::UpdatePosition(float frame_time)
{
	// Add the accelerativew forces together
	accel_ = *separation_ + *cohesion_ + *alignment_;
	// s = ut + 1/2(a(t^2))
	curr_pos_ = (prev_pos_ * frame_time) + ((accel_*pow(frame_time, 2)) / 2);

	gef::Vector4 pos = gef::Vector4(curr_pos_.x, curr_pos_.y, 0.0f);
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
