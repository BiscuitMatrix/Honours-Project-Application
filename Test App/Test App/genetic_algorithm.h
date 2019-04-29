#pragma once

#include "Globals.h"

#include "boid.h"
#include "DNA.h"

struct Population
{
	float sum_fitness_;
	float data[12];
	std::string id_;
};

class genetic_algorithm
{
public:
	genetic_algorithm();
	~genetic_algorithm();

	// Initialise the algorithm
	void InitialiseFromFiles();
	void Initialise(std::vector<boid>*);
	void Heuristic(float* data);
	// Update the Algorithm
	void Update(std::vector<boid>*, float, float, int);
	// Evaluate the population at the end of each simulation
	void Evaluation(float , float);
	// Select top boids from the population
	void Selection(int, float, float);
	void susSelection(int, float, float);

	void CloseFiles();

	std::vector<boid>* GetBoids() { return boids_; }

private:
	int generation_;
	int population_;

	int num_of_genotypes_;

	std::vector<boid>* boids_;
	std::vector<Population> populations_;
	int mutation_rate_;

	float boid_health_;
	float flock_health_;
	float competing_flock_health_;

	// Fitness Statistics:
	float highest_fitness_;
	float lowest_fitness_;
	float range_fitness_;
	float mean_fitness_;
	float sum_fitness_;

	std::ofstream excel_data_[glo_flock_size];
};