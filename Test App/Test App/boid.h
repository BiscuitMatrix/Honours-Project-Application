#pragma once

#include <system/application.h>
#include <graphics/mesh_instance.h>
#include <graphics/mesh.h>
#include <graphics/scene.h>
#include <graphics/primitive.h>
#include <graphics/material.h>

#include <maths/vector4.h>
#include <maths/vector2.h>
#include <maths/math_utils.h>

class boid
{
public:
	boid(gef::Platform& platform);
	~boid();

	void Initialise();

	void Update(float frame_time);

	static void RunBoidsAlgorithm(std::vector<boid>* boid, float frame_time);

	void CollDetect();
	void Bounds(float x, float z);
	void WrapAround(float x, float z);

	void UpdatePosition(float frame_time);

	void CleanUp();

	gef::MeshInstance* GetMeshInstance() { return cube_; };

	// Position Mutators
	gef::Vector2 GetPos() { return pos_; };
	void SetPos(gef::Vector2 new_pos) { pos_ = new_pos; };
	// Velocity Mutators
	gef::Vector2 GetVel() { return vel_; };
	void SetVel(gef::Vector2 new_vel) { vel_ = new_vel; };
	// Acceleration Mutators
	gef::Vector2 GetAccel() { return accel_; };
	void SetAccel(gef::Vector2 new_accel) { accel_ = new_accel; };


	void SetTranslation(gef::Vector2 pos) 
	{ 
		gef::Vector4 v4_pos = gef::Vector4(pos.x, 0.0f, pos.y);
		translation_.SetTranslation(v4_pos);
		pos_ = pos;
	};
	gef::Matrix44 GetTranslation() { return translation_; };
	gef::Matrix44 GetScale() { return scale_; };
	gef::Matrix44 GetRotation() { return rotation_; };


	gef::MeshInstance* GetCube() { return cube_; };

private:
	gef::Mesh* CreateCubeMesh();

	class gef::Mesh* mesh_;
	gef::MeshInstance* cube_;
	UInt32 colour_;

	// Linear Motion Variables (SUVAT)
	gef::Vector2 accel_;
	gef::Vector2 vel_;
	gef::Vector2 pos_;

	// Transformation Matrices
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Platform& platform_;
};

