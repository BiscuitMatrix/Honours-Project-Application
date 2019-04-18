#include "genetic_algorithm.h"



genetic_algorithm::genetic_algorithm() :
	generation_(0),
	population_(glo_flock_size)
{
}


genetic_algorithm::~genetic_algorithm()
{
}


void genetic_algorithm::Initialise(std::vector<boid> *boids)
{
	// Give the genetic algorithm a reference to the boids in the scene
	boid_ref_ = boids;

	// populate the genetic data
	for (int i = 0; i < glo_flock_size; i++)
	{
		DNA* dna = new DNA();

		float data[12];
		float genetic_variation;
		// Cohesion Variables
		data[0] = 1.0f;			// coh_mult_
		data[1] = 30.0f;		// coh_div_mult_
		// Alignment Variables
		data[2] = 1.0f;			// ali_mult_
		data[3] = 10.0f;		// ali_div_mult_
		// Separation Variables
		data[4] = 0.025f;		// sep_mult_
		data[5] = 1.0f;			// sep_neigh_mult_
		data[6] = 1.0f;			// sep_clo_neigh_mult_
		// Food Attraction Variables
		data[7] = 0.0025f;		// food_mult_1_
		data[8] = 1.0f;			// food_mult_2_
		data[9] = 1.0f;			// food_div_mult_
		// Flock Avoidance Variables
		data[10] = 300.0f;		// floav_mult_
		data[11] = 1.0f;		// floav_div_mult_
		
		dna->UpdateDataSet(data);
		genetic_information_.push_back(*dna);
	}
	for (std::vector<DNA>::iterator iterator = genetic_information_.begin(); iterator != genetic_information_.end(); iterator++)
	{
		// The DNA will affect the weightings of each boid force in a standardised way (but will create a lot of variety!)

	}

	// Add the genetic data to these boids
	std::vector<DNA>::iterator dna_iterator = genetic_information_.begin();
	for (std::vector<boid>::iterator iterator = boid_ref_->begin(); iterator != boid_ref_->end(); iterator++)
	{
		// Set the DNA for that boid
		//iterator->SetDNA(*dna_iterator);
		// Move the DNA iterator along (assumes DNA vector length is the same as boid vector length
		//if (dna_iterator != genetic_information_.end())
		//{
		//	dna_iterator++;
		//}
	}

	//boids_ = nullptr;
}

void genetic_algorithm::Evaluate()
{

}
