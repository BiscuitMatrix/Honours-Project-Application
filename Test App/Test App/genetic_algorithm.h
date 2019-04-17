#pragma once

#include "Globals.h"

#include "DNA.h"



class genetic_algorithm
{
public:
	genetic_algorithm();
	~genetic_algorithm();

	// Initialise the algorithm
	void Initialise();
	// Evaluate the population at the end of each simulation
	void Evaluate();
	// Select top boids from the population
	void Selection();
	// Cross-over "mix" the genetic data
	void Crossover();

	// Take a sample at set intervals of the genetic data of the populace
	void GeneticSnapshot();

	void CleanUp();

private:
	int generation_;
	int population_;

	DNA dna_[glo_flock_size];

	float boid_health_;
	float flock_health_;
	float competing_flock_health_;
};

