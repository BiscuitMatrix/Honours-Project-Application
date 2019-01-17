#pragma once

#include <system/application.h>
#include <graphics/mesh_instance.h>
#include <graphics/mesh.h>
#include <graphics/primitive.h>

struct Vector2
{
	float x;
	float y;
};


class boid
{
public:
	boid(gef::Platform& platform);
	~boid();

	void Initialise();

	void Update(float frame_time);

	void UpdatePosition(float frame_time);

	void CleanUp();

	gef::MeshInstance GetMeshInstance() { return cube_; };

private:
	gef::Mesh* CreateCubeMesh();
	class gef::Mesh* mesh_;
	gef::MeshInstance cube_;


	// Linear Motion Variables (SUVAT)
	Vector2 accel_;
	Vector2 prev_vel_;
	Vector2 curr_vel_;
	Vector2 prev_pos_;
	Vector2 curr_pos_;
	Vector2 displacement_;


	// 
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Platform& platform_;
};

