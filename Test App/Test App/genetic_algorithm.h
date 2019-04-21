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
	void Update(std::vector<boid>*, float, float, int);
	// Evaluate the population at the end of each simulation
	void Evaluation(std::vector<boid>*, float , float);
	// Select top boids from the population
	void Selection(std::vector<boid>*);
	void susSelection(std::vector<boid>*);
	void AboveAverageSelection(std::vector<boid>*);
	// Cross-over "mix" the genetic data
	void Crossover();
	// Mutate the new gene pool
	void Mutation(std::vector<boid>*, int);

	// Take a sample at set intervals of the genetic data of the populace
	void GeneticSnapshot();

private:
	int generation_;
	int population_;

	std::vector<boid> top_boid_strains_;
	//std::vector<DNA> genetic_information_;

	float boid_health_;
	float flock_health_;
	float competing_flock_health_;

	// Fitness Statistics:
	float highest_fitness_;
	float lowest_fitness_;
	float range_fitness_;
	float mean_fitness_;
};

