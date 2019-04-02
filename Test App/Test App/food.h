#pragma once

#include <vector>
#include "resource.h"

class food
{
public:
	food(gef::Platform& platform);
	~food();

	void Initialise(int resource_count);
	void Update(float frame_time);

	void CleanUp();

	std::vector<resource> resources_;

private:
	static float bound_x_, bound_y_;

	gef::Platform& platform_;
};

