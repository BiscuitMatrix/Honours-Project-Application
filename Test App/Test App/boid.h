#pragma once

#include <system/application.h>
#include <graphics/mesh_instance.h>
#include <graphics/mesh.h>
#include <graphics/scene.h>
#include <graphics/primitive.h>
#include <graphics/material.h>

#include <maths/vector4.h>
#include <maths/vector2.h>

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
	gef::Vector2 GetCurrPos() { return curr_pos_; };
	void SetCurrPos(gef::Vector2 new_curr_pos) { curr_pos_ = new_curr_pos; };
	gef::Vector2 GetPrevPos() { return prev_pos_; };
	void SetPrevPos(gef::Vector2 new_prev_pos) { curr_pos_ = new_prev_pos; };
	// Velocity Mutators
	gef::Vector2 GetCurrVel() { return curr_vel_; };
	void SetCurrVel(gef::Vector2 new_curr_vel) { curr_vel_ = new_curr_vel; };
	gef::Vector2 GetPrevVel() { return prev_vel_; };
	void SetPrevVel(gef::Vector2 new_prev_vel) { curr_vel_ = new_prev_vel; };
	// Acceleration Mutators
	gef::Vector2 GetAccel() { return accel_; };
	void SetAccel(gef::Vector2 new_accel) { accel_ = new_accel; };
	// Displacement Mutators
	gef::Vector2 GetDisplacement() { return displacement_; };
	void SetDisplacement(gef::Vector2 new_displacement) { displacement_ = new_displacement; };


	void SetTranslation(gef::Vector2 pos) 
	{ 
		gef::Vector4 v4_pos = gef::Vector4(pos.x, 0.0f, pos.y);
		translation_.SetTranslation(v4_pos);
		curr_pos_.x = translation_.m(3, 0);
		curr_pos_.y = translation_.m(3, 2);
	};
	gef::Matrix44 GetTranslation() { return translation_; };
	gef::Matrix44 GetScale() { return scale_; };
	gef::Matrix44 GetRotation() { return rotation_; };

	gef::MeshInstance* GetCube() { return cube_; };

	UInt32 GetColour() { return colour_; }
	void SetColour(UInt32 abgr) { colour_ = abgr; }

private:
	gef::Mesh* CreateCubeMesh();

	class gef::Mesh* mesh_;
	gef::MeshInstance* cube_;
	UInt32 colour_;

	// Linear Motion Variables (SUVAT)
	gef::Vector2 accel_;
	gef::Vector2 prev_vel_, curr_vel_;
	gef::Vector2 prev_pos_, curr_pos_;
	gef::Vector2 displacement_;

	// Transformation Matrices
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Platform& platform_;

	gef::Material* material_;
};

