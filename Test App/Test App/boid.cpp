#include "boid.h"

boid::boid(gef::Platform& platform) :
	platform_(platform),
	mesh_(nullptr),
	cube_(nullptr),
	vel_(0.0f, 0.0f),
	health_(glo_boid_max_health),
	hunger_(glo_boid_max_hunger)
{
}

boid::~boid()
{
}

void boid::Initialise()
{
	hunger_ = glo_boid_max_hunger;

	mesh_ = new gef::Mesh(platform_);
	mesh_ = CreateCubeMesh();

	cube_ = new gef::MeshInstance();
	cube_->set_mesh(mesh_);

	scale_.SetIdentity();
	rotation_.SetIdentity();
	translation_.SetIdentity();
}

void boid::Update(float frame_time)
{
	hunger_ -= (1.0f * frame_time);
}


void boid::Bounds(float x, float z)
{
	if (pos_.x > x)
	{
		pos_.x = x;
		vel_.x = 0.0f;
	}
	if (pos_.x < -x)
	{
		pos_.x = -x;
		vel_.x = 0.0f;
	}
	if (pos_.y > z)
	{
		pos_.y = z;
		vel_.y = 0.0f;
	}
	if (pos_.y < -z)
	{
		pos_.y = -z;
		vel_.y = 0.0f;
	}
}

void boid::WrapAround(float x, float z)
{
	if (pos_.x < -x) pos_.x = x;
	if (pos_.y < -z) pos_.y = z;
	if (pos_.x > x) pos_.x = -x;
	if (pos_.y > z) pos_.y = -z;
}

void boid::CleanUp()
{
	delete mesh_;
	mesh_ = nullptr;

	delete cube_;
	cube_ = nullptr;
}

void boid::HasEaten()
{
	hunger_ += 50.0f;
	//if (health_)
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
		//// Back
		//0, 1, 2,
		//2, 3, 0,
		//// Front
		//6, 5, 4,
		//4, 7, 6,
		//// Left
		//2, 7, 3,
		//2, 6, 7,
		// Right
		//0, 4, 1,
		//5, 1, 4,
		// Top
		6, 2, 1,
		5, 6, 1
		// Bottom
		//0, 3, 7,
		//0, 7, 4
	};

	primitive->InitIndexBuffer(platform_, static_cast<const void*>(indices), sizeof(indices) / sizeof(UInt32), sizeof(UInt32));
	primitive->set_type(gef::TRIANGLE_LIST);

	// We can make the half-size smaller here to make the collision detection occur a lot closer.

	// set size of bounds, we need this for collision detection routines
	gef::Aabb aabb(gef::Vector4(-half_size, -half_size, -half_size), gef::Vector4(half_size, half_size, half_size));
	gef::Sphere sphere(aabb);

	mesh->set_aabb(aabb);
	mesh->set_bounding_sphere(sphere);

	return mesh;
}