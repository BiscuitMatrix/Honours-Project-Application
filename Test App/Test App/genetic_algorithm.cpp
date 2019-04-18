#include "genetic_algorithm.h"



genetic_algorithm::genetic_algorithm() :
	generation_(0),
	population_(glo_flock_size)
{
}


genetic_algorithm::~genetic_algorithm()
{
}


void genetic_algorithm::Initialise(std::vector<boid> boids)
{
	// Give the genetic algorithm a reference to the boids in the scene
	//boids_ = boids;

	// populate the genetic data
	for (std::vector<DNA>::iterator iterator = genetic_information_.begin(); iterator != genetic_information_.end(); iterator++)
	{
		// The DNA will affect the weightings of each boid force in a standardised way (but will create a lot of variety!)

	}

	// Add the genetic data to these boids
	std::vector<DNA>::iterator dna_iterator = genetic_information_.begin();
	for (std::vector<boid>::iterator iterator = boids_.begin(); iterator != boids_.end(); iterator++)
	{
		// Set the DNA for that boid
		iterator->SetDNA(*dna_iterator);
		// Move the DNA iterator along (assumes DNA vector length is the same as boid vector length
		if (dna_iterator != genetic_information_.end())
		{
			dna_iterator++;
		}
	}

}

void genetic_algorithm::Evaluate()
{

}
