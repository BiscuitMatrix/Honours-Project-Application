#pragma once

#include <graphics/mesh.h>
#include <graphics/primitive.h>

class resource
{
public:
	resource(gef::Platform& platform);
	~resource();

	void SetActive(bool active) { active_ = active; };
	bool GetActive() { return active_; };

	void SetPos(gef::Vector2 pos) { pos_ = pos; };
	gef::Vector2 GetPos() { return pos_; };

	gef::Mesh* CreateCubeMesh();

private:
	bool active_;

	gef::Vector2 pos_;

	gef::Platform& platform_;
};

