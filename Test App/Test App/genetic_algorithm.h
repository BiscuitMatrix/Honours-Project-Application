#pragma once

#include "Globals.h"

#include "boid.h"
#include "DNA.h"



class genetic_algorithm
{
public:
	genetic_algorithm();
	~genetic_algorithm();

	// Initialise the algorithm
	void Initialise(std::vector<boid>*);
	// Update the Algorithm
	void Update(std::vector<boid>*);

	// Evaluate the population at the end of each simulation
	void Evaluate();
	// Select top boids from the population
	void Selection();
	// Cross-over "mix" the genetic data
	void Crossover();

	// Take a sample at set intervals of the genetic data of the populace
	void GeneticSnapshot();

	void CleanUp();

	std::vector<boid>* boid_ref_;

private:
	int generation_;
	int population_;

	

	std::vector<DNA> genetic_information_;

	float boid_health_;
	float flock_health_;
	float competing_flock_health_;
};

