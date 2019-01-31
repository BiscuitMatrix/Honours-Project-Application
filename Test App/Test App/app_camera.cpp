#include "app_camera.h"



app_camera::app_camera()
{
}


app_camera::~app_camera()
{
}

void app_camera::SetupCamera()
{
	// initialise the camera settings
	camera_eye = gef::Vector4(0.0f, 80.0f, 0.0001f);
	camera_lookat = gef::Vector4(0.0f, 0.0f, 0.0f);
	camera_up = gef::Vector4(0.0f, 1.0f, 0.0f);
	camera_fov = gef::DegToRad(45.0f);
	near_plane = 0.01f;
	far_plane = 100000.f;
}
