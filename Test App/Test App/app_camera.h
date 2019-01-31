#pragma once

#include <maths/vector4.h>
#include <maths/math_utils.h>


class app_camera
{
public:
	app_camera();
	~app_camera();

	void SetupCamera();

	gef::Vector4 camera_eye;
	gef::Vector4 camera_lookat;
	gef::Vector4 camera_up;
	float camera_fov;
	float near_plane;
	float far_plane;
};

