#pragma once

#include <system/application.h>
#include <graphics/mesh_instance.h>
#include <graphics/mesh.h>
#include <graphics/scene.h>
#include <graphics/primitive.h>

#include <maths/vector2.h>

class boid
{
public:
	boid(gef::Platform& platform);
	~boid();

	void Initialise();

	void Update(float frame_time);

	void RunBoidsAlgorithm(std::vector<boid>* boid, float frame_time);

	void UpdatePosition(float frame_time);

	void CleanUp();

	gef::MeshInstance* GetMeshInstance() { return cube_; };

	gef::Vector2 GetCurrPos() { return curr_pos_; };

	void SetSeparationVector(gef::Vector2* sep) { separation_ = sep; };
	void SetCohesionVector(gef::Vector2* coh) { cohesion_ = coh; };
	void SetAlignmentVector(gef::Vector2* ali) { alignment_ = ali; };

private:
	gef::Mesh* CreateCubeMesh();

	class gef::Mesh* mesh_;
	gef::MeshInstance* cube_;

	std::vector<boid>::iterator iterator_;
	std::vector<boid>::iterator iterator_2_;

	// Reynolds Forces
	gef::Vector2 *separation_, *cohesion_, *alignment_;
	float desired_separation_;
	// Reynolds Weights
	float sep_wgt_, coh_wgt_, ali_wgt;
	// Reynolds Counters
	int sep_counter_, ali_counter_, coh_counter_;
	// Limits
	float max_force_, max_speed_;

	int id_;

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
};

