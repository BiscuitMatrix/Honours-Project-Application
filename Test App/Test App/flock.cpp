#include "flock.h"



flock::flock(gef::Platform& platform) :
	platform_(platform)
{
}

flock::~flock()
{
}


void flock::Initialise(int flock_size)
{
	boid_ref_ = new boid(platform_);
	boid_ref_->Initialise();

	boids_ = new std::vector<boid>();
	boids_->begin();

	for (int i = 0; i < flock_size; i++)
	{
		boid* boid_ = new boid(platform_);
		boid_->Initialise();
		float x = ((float)i / 5.0f) * 3.14159f;
		float y = ((float)i / 5.0f) * 3.14159f;
		gef::Vector4 pos;
		if (i <= 10)
		{
			pos = gef::Vector4(sin(x), 0.0f, cos(y));
		}
		else if (i > 10 && i <= 20)
		{
			pos = gef::Vector4(2.0f*sin(x), 0.0f, 2.0f*cos(y));
		}
		else if (i > 20 && i <= 30)
		{
			pos = gef::Vector4(3.0f*sin(x), 0.0f, 3.0f*cos(y));
		}
		boid_->SetTranslation(pos);
		boid_->GetMeshInstance()->set_transform(boid_->GetTranslation());
		boids_->push_back(*boid_);
	}
	int size = boids_->size();
}

void flock::Update(float frame_time)
{
	// Run boids algorithm
	boid_ref_->RunBoidsAlgorithm(boids_, frame_time);
	
	// Run the Genetic Algorithm

}

void flock::RunBoidsAlgorithm()
{
	
}

void flock::CleanUp()
{
}
