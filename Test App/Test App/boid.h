#pragma once

#include <string>

#include <system/application.h>

#include <graphics/mesh_instance.h>
#include <graphics/mesh.h>
#include <graphics/scene.h>
#include <graphics/primitive.h>
#include <graphics/material.h>

#include <maths/vector4.h>
#include <maths/vector2.h>
#include <maths/math_utils.h>

#include "Globals.h"

#include "DNA.h"

class boid
{
public:
	boid(gef::Platform& platform);
	~boid();

	void Initialise();

	void Update(float frame_time);

	void Bounds(float x, float z);
	void WrapAround(float x, float z);

	void CleanUp();

	void HasEaten();

	// Position Mutators
	gef::Vector2 GetPos() { return pos_; };
	gef::Matrix44 GetTranslationMatrix() { return translation_; };
	void SetPos(gef::Vector2 pos)
	{
		gef::Vector4 v4_pos = gef::Vector4(pos.x, 0.0f, pos.y);
		translation_.SetTranslation(v4_pos);
		pos_ = pos;
	};
	// Velocity Mutators
	gef::Vector2 GetVel() { return vel_; };
	void SetVel(gef::Vector2 new_vel) { vel_ = new_vel; };
	// Framework Mutators
	gef::Mesh* GetMesh() { return mesh_; };
	gef::MeshInstance* GetMeshInstance() { return cube_; };

	// Health Mutators
	int GetHealth() { return health_; }
	void ResetHealth(int health) { health_ = glo_boid_max_health; }
	void AddHealth(int add_health)
	{
		if ((health_ + add_health) > glo_boid_max_health) health_ = glo_boid_max_health;
		else health_ += add_health;
	}
	float GetHunger() { return hunger_; }
	void DecreaseHunger(float sub_hunger)
	{
		if ((hunger_ + sub_hunger) > glo_boid_max_hunger) hunger_ = glo_boid_max_hunger;
		else hunger_ += sub_hunger;
	}

	// Genetic Algorithm Mutators
	DNA GetDNA() { return dna_; }
	void SetDNA(DNA dna) { dna_ = dna; }
	float GetFitness() { return fitness_; }
	void SetFitness(float fitness) { fitness_ = fitness; }
	std::string GetPopID() { return pop_id_; }
	void SetPopID(std::string pop_id) { pop_id_ = pop_id; }

private:
	gef::Mesh* CreateCubeMesh();

	// Render Properties:
	class gef::Mesh* mesh_;
	gef::MeshInstance* cube_;
	UInt32 colour_;

	// Health variables: (Basically a motive to stay alive)
	int health_;
	float hunger_;
	bool active_;

	// Genetic Algorithm Variables:
	DNA dna_;
	float fitness_;
	std::string pop_id_;

	// Linear Motion Variables:
	gef::Vector2 vel_;
	gef::Vector2 pos_;

	// Transformation Matrices
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Platform& platform_;
};

