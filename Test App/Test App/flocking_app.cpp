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

#include <thread>


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
	srand(time(NULL));

	// These are required for the setup of the 3D scene.
	sprite_renderer_ = gef::SpriteRenderer::Create(platform_);
	renderer_3d_ = gef::Renderer3D::Create(platform_);

	InitFont();

	// Environment setup
	environment_ = new gef::Sprite();
	environment_->set_position(960.0f / 2.0f, 544.0f / 2.0f, 100.0f);
	environment_->set_width(940.0f);
	environment_->set_height(524.0f);

	// Flock 1 settings
	flock_1_ = new flock(platform_, false);
	flock_1_pos_ = gef::Vector2(-40.0f, 20.0f);
	//flock_1_pos_ = gef::Vector2(-10.0f, 5.0f);
	flock_1_->Initialise(flock_1_pos_);

	// Flock 2 settings
	flock_2_ = new flock(platform_, true);
	flock_2_pos_ = gef::Vector2(40.0f, -20.0f);
	//flock_2_pos_ = gef::Vector2(10.0f, -5.0f);
	flock_2_->Initialise(flock_2_pos_);

	// Food settings
	food_ = new food(platform_);
	resource_count_ = 20;
	food_->Initialise(resource_count_);

	// Camera Setup
	cam_1_.SetupCamera();
	SetupLights();

	// Genetic Algorithm Setup
	genetic_improvement_ = true;
	generation_ = 1;
	ga_timecheck_ = true;
}

void flocking_app::CleanUp()
{
	delete genetic_algorithm_;
	genetic_algorithm_ = nullptr;

	flock_1_->CleanUp();
	delete flock_1_;
	flock_1_ = nullptr;

	flock_2_->CleanUp();
	delete flock_2_;
	flock_2_ = nullptr;

	food_->CleanUp();
	delete food_;
	food_ = nullptr;

	delete environment_;
	environment_ = nullptr;

	CleanUpFont();
	delete sprite_renderer_;
	sprite_renderer_ = NULL;

	delete renderer_3d_;
	renderer_3d_ = NULL;
}

bool flocking_app::Update(float frame_time)
{
	fps_ = 1.0f / frame_time;

	// Start the timer - This will allow me to record how much faster than time this project works
	auto start_time = clock_::now();

	// Here is where we can get the GA to run without rendering if we keep everything enclosed in this function
	int generations_left_ = glo_number_of_generations_per_improvement;
	float time_per_generation = 120.0f;
	// Skip the rendering process while this occurs
	while (genetic_improvement_)
	{
		// Run, run, as fast as you can! You can't catch meee, i'm the gingerbread man!

		if (rand() % 2 == 1)
		{
			flock_1_->Update(&flock_2_->boids_, &food_->resources_, glo_ga_frame_time);
			flock_2_->Update(&flock_1_->boids_, &food_->resources_, glo_ga_frame_time);
		}
		else
		{
			flock_2_->Update(&flock_1_->boids_, &food_->resources_, glo_ga_frame_time);
			flock_1_->Update(&flock_2_->boids_, &food_->resources_, glo_ga_frame_time);
		}

		food_->Update(frame_time);

		time_per_generation -= glo_ga_frame_time;

		if (time_per_generation <= 0.0f)
		{
			// Reset time for the next generation
			time_per_generation = 120.0f;
			// Reduce the number of generations
			generations_left_--;
			// Increase the number of generations in this simulation
			generation_++;
			// Reset the simulation
			flock_1_->Reset(generation_);
			flock_2_->GAReset(flock_1_->GetFlockHealth(), generation_);
		}

		if (generations_left_ == 0)
		{
			flock_2_->CloseFiles();
			genetic_improvement_ = false;
		}
	}

	auto end_time = clock_::now();
	double duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	if (ga_timecheck_)
	{
		ga_timecheck_ = false;
	}
	
	//if (button_press)
	//{
		// Reset the simulation
		// ...
		//
		// Run the genetic improvement again
		// genetic_improvement = true;
	//}

	float run_speed_dt = frame_time;

	// Regular Rendered Simulation
	if (rand() % 2 == 1)
	{
		flock_1_->Update(&flock_2_->boids_, &food_->resources_, run_speed_dt);
		flock_2_->Update(&flock_1_->boids_, &food_->resources_, run_speed_dt);
	}
	else
	{
		flock_2_->Update(&flock_1_->boids_, &food_->resources_, run_speed_dt);
		flock_1_->Update(&flock_2_->boids_, &food_->resources_, run_speed_dt);
	}

	food_->Update(frame_time);

	return true;
}

void flocking_app::Render()
{
	gef::Matrix44 projection_matrix;
	//gef::Matrix44 orthographic_matrix;
	gef::Matrix44 view_matrix;

	gef::Material object_colour;
	UInt32 colour;

	//orthographic_matrix = platform_.OrthographicFrustum();
	projection_matrix = platform_.PerspectiveProjectionFov(cam_1_.camera_fov, (float)platform_.width() / (float)platform_.height(), cam_1_.near_plane, cam_1_.far_plane);
	view_matrix.LookAt(cam_1_.camera_eye, cam_1_.camera_lookat, cam_1_.camera_up);
	renderer_3d_->set_projection_matrix(projection_matrix);
	renderer_3d_->set_view_matrix(view_matrix);

	// draw meshes here
	renderer_3d_->Begin();
	
	// Set the colour of flock 1 - Red
	colour = 0xFF0000FF;
	object_colour.set_colour(colour);
	renderer_3d_->set_override_material(&object_colour);
	// Iterate through the vector list of boids within the flock in order to render them all
	for (std::vector<boid>::iterator iterator_ = flock_1_->boids_.begin(); iterator_ != flock_1_->boids_.end(); iterator_++)
	{
		if(iterator_->GetActive()) renderer_3d_->DrawMesh(*iterator_->GetMeshInstance());
	}

	// Set the colour of the flock 2 - Blue
	colour = 0xFFFF0000;
	object_colour.set_colour(colour);
	renderer_3d_->set_override_material(&object_colour);
	// Iterate through the vector list of boids within the flock in order to render them all
	for (std::vector<boid>::iterator iterator_ = flock_2_->boids_.begin(); iterator_ != flock_2_->boids_.end(); iterator_++)
	{
		if (iterator_->GetActive()) renderer_3d_->DrawMesh(*iterator_->GetMeshInstance());
	}

	// Set the colour of the food - Green
	colour = 0xFF00FF00;
	object_colour.set_colour(colour);
	renderer_3d_->set_override_material(&object_colour);
	// Iterate through the vector list of boids within the flock in order to render them all
	for (std::vector<resource>::iterator iterator_ = food_->resources_.begin(); iterator_ != food_->resources_.end(); iterator_++)
	{
		renderer_3d_->DrawMesh(*iterator_->GetMeshInstance());
	}

	renderer_3d_->End();

	// setup the sprite renderer, but don't clear the frame buffer
	// draw 2D sprites here
	sprite_renderer_->Begin(false);
	sprite_renderer_->DrawSprite(*environment_);
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
		font_->RenderText(sprite_renderer_, gef::Vector4(850.0f, 510.0f, -0.9f), 1.0f, 0xffff00ff, gef::TJ_LEFT, "FPS: %.1f", fps_);
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




