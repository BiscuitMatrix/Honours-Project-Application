#pragma once

#include <graphics/mesh.h>
#include <graphics/mesh_instance.h>
#include <graphics/primitive.h>

class resource
{
public:
	resource(gef::Platform& platform);
	~resource();

	void Initialise();
	void CleanUp();

	void SetActive(bool active) { active_ = active; };
	bool GetActive() { return active_; };

	gef::MeshInstance* GetMeshInstance() { return cube_; };

	void SetPos(gef::Vector2 pos) 
	{ 
		gef::Vector4 v4_pos = gef::Vector4(pos.x, 0.0f, pos.y);
		translation_.SetTranslation(v4_pos);
		pos_ = pos; 
	};
	gef::Vector2 GetPos() { return pos_; };
	gef::Matrix44 GetTranslationMatrix() { return translation_; };

private:
	bool active_;
	class gef::Mesh* mesh_;
	gef::MeshInstance* cube_;
	UInt32 colour_;
	gef::Mesh* CreateCubeMesh();

	// Transformation Matrices
	gef::Matrix44 scale_;
	gef::Matrix44 rotation_;
	gef::Matrix44 translation_;

	gef::Vector2 pos_;

	gef::Platform& platform_;
};

