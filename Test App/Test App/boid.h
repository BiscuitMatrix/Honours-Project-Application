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

	void CleanUp();

	gef::MeshInstance GetMeshInstance() { return cube_; };

private:
	gef::Mesh* CreateCubeMesh();

	class gef::Mesh* mesh_;
	gef::MeshInstance cube_;

	Vector2 pos_;
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Platform& platform_;
};

