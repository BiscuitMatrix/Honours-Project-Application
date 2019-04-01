#pragma once

#include "resource.h"

class food
{
public:
	food(gef::Platform& platform);
	~food();

	void Initialise(int resource_count);

private:

	gef::Platform& platform_;
};

