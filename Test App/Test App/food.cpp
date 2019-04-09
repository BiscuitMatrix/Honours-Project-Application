#include "food.h"

float food::bound_x_ = 55.0f;
float food::bound_y_ = 30.0f;

food::food(gef::Platform& platform) :
	platform_(platform)
{
}


food::~food()
{
}

void food::Initialise(int resource_count)
{
	for (int i = 0; i < resource_count; i++)
	{
		resource resource_(platform_);
		resource_.Initialise();

		float x = (float)(rand() % (int)(bound_x_*2.0f + 1)) - bound_x_;
		float y = (float)(rand() % (int)(bound_y_*2.0f + 1)) - bound_y_;
		
		resource_.SetPos(gef::Vector2(x,y));

		resource_.GetMeshInstance()->set_transform(resource_.GetTranslationMatrix());

		resources_.push_back(resource_);
	}
}

void food::Update(float frame_time)
{
	// This will update the position of resources located within the simulation
	// For each resource in the environment:
	for (std::vector<resource>::iterator iterator = resources_.begin(); iterator != resources_.end(); iterator++)
	{
		if (!iterator->GetActive())
		{
			iterator->SetActive(true);

			float x = (float)(rand() % (int)(bound_x_*2.0f + 1)) - bound_x_;
			float y = (float)(rand() % (int)(bound_y_*2.0f + 1)) - bound_y_;

			iterator->SetPos(gef::Vector2(x, y));
			iterator->GetMeshInstance()->set_transform(iterator->GetTranslationMatrix());
		}
	}
}

void food::CleanUp()
{
	for (std::vector<resource>::iterator iterator_ = resources_.begin(); iterator_ != resources_.end(); iterator_++)
	{
		iterator_->CleanUp();
	}
}


