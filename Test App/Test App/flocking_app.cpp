#include "flocking_app.h"
#include <system/platform.h>
#include <graphics/sprite_renderer.h>
#include <graphics/texture.h>
#include <graphics/mesh.h>
#include <graphics/primitive.h>
#include <assets/png_loader.h>
#include <graphics/image_data.h>
#include <graphics/font.h>
#include <input/touch_input_manager.h>
#include <maths/vector2.h>
#include <input/sony_controller_input_manager.h>
#include <maths/math_utils.h>
#include <graphics/renderer_3d.h>


flocking_app::flocking_app(gef::Platform& platform) :
	Application(platform),
	sprite_renderer_(NULL),
	font_(NULL),
	renderer_3d_(NULL)
{
}
flocking_app::~flocking_app()
{
}

void flocking_app::Init()
{
	// These are required for the setup of the 3D scene.
	sprite_renderer_ = gef::SpriteRenderer::Create(platform_);
	renderer_3d_ = gef::Renderer3D::Create(platform_);

	InitFont();

	flock_1_ = new flock(platform_);
	flock_size_1_ = 2;
	flock_1_->Initialise(flock_size_1_);

	SetupCamera();
	SetupLights();
}

void flocking_app::CleanUp()
{
	flock_1_->CleanUp();
	delete flock_1_;
	flock_1_ = nullptr;

	CleanUpFont();
	delete sprite_renderer_;
	sprite_renderer_ = NULL;

	delete renderer_3d_;
	renderer_3d_ = NULL;
}

bool flocking_app::Update(float frame_time)
{
	fps_ = 1.0f / frame_time;

	return true;
}

void flocking_app::Render()
{
	gef::Matrix44 projection_matrix;
	gef::Matrix44 view_matrix;

	projection_matrix = platform_.PerspectiveProjectionFov(camera_fov, (float)platform_.width() / (float)platform_.height(), near_plane, far_plane);
	view_matrix.LookAt(camera_eye, camera_lookat, camera_up);
	renderer_3d_->set_projection_matrix(projection_matrix);
	renderer_3d_->set_view_matrix(view_matrix);

	// draw meshes here
	renderer_3d_->Begin();
	// Iterate through the vector list of boids within the flock in order to render them all
	for (flock_1_->iterator_ = flock_1_->boids_.begin(); flock_1_->iterator_ != flock_1_->boids_.end(); flock_1_->iterator_++)
	{
		renderer_3d_->DrawMesh(flock_1_->iterator_->GetMeshInstance());
	}
	//renderer_3d_->DrawMesh(boid_->GetMeshInstance());
	renderer_3d_->End();

	// setup the sprite renderer, but don't clear the frame buffer
	// draw 2D sprites here
	sprite_renderer_->Begin(false);
	DrawHUD();
	sprite_renderer_->End();
}

void flocking_app::InitFont()
{
	font_ = new gef::Font(platform_);
	font_->Load("comic_sans");
}
void flocking_app::CleanUpFont()
{
	delete font_;
	font_ = NULL;
}
void flocking_app::DrawHUD()
{
	if (font_)
	{
		// display frame rate
		font_->RenderText(sprite_renderer_, gef::Vector4(850.0f, 510.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "FPS: %.1f", fps_);
	}
}

void flocking_app::SetupLights()
{
	gef::PointLight default_point_light;
	default_point_light.set_colour(gef::Colour(0.7f, 0.7f, 1.0f, 1.0f));
	default_point_light.set_position(gef::Vector4(-500.0f, 400.0f, 700.0f));

	gef::Default3DShaderData& default_shader_data = renderer_3d_->default_shader_data();
	default_shader_data.set_ambient_light_colour(gef::Colour(0.5f, 0.5f, 0.5f, 1.0f));
	default_shader_data.AddPointLight(default_point_light);
}

void flocking_app::SetupCamera()
{
	// initialise the camera settings
	camera_eye = gef::Vector4(5.0f, 5.0f, 5.0f);
	camera_lookat = gef::Vector4(0.0f, 0.0f, 0.0f);
	camera_up = gef::Vector4(0.0f, 1.0f, 0.0f);
	camera_fov = gef::DegToRad(45.0f);
	near_plane = 0.01f;
	far_plane = 1000.f;
}


