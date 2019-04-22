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
	int file_num = 0;
	// For each boid:
	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// populate the genetic data
			DNA dna = DNA();

			float data[12];

			// Intrduce a random amount of variation into the genetic data: (genotype)
			for (int data_point = 0; data_point < 12; data_point++)
			{
				data[data_point] = rand() % 1000;
			}

			//Heuristic(data);

			dna.UpdateDataSet(data);

			std::string txt_directory("GeneticDatatxt/Generation_1");
			std::string txt_file_name("/boid_data_");
			std::string txt_base(".txt");

			std::string csv_directory("GeneticDataCSV/Simulation_");
			std::string csv_file_name("/boid_data_");
			std::string csv_base(".csv");

			dna.StoreData(txt_directory+txt_file_name+std::to_string(file_num)+txt_base, csv_directory+ std::to_string(glo_simulation_number)+csv_file_name+std::to_string(file_num)+csv_base);
			file_num++;

			iterator->SetDNA(dna);
	}
}

void genetic_algorithm::Heuristic(float* data)
{
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
}

void genetic_algorithm::Update(std::vector<boid>* boids, float flock_health, float enemy_flock_health, int generation)
{
	Evaluation	(boids, flock_health, enemy_flock_health);
	Selection	(boids);
	Crossover	();
	Mutation	(boids, generation);
}

// Maybe also collect data here too, this will be useful in terms of getting useful data:
void genetic_algorithm::Evaluation(std::vector<boid>* boids, float flock_health, float enemy_flock_health)
{
	highest_fitness_ = 0.0f;
	lowest_fitness_ = 100000000.0f;
	range_fitness_ = 0.0f;
	mean_fitness_ = 0.0f;
	sum_fitness_ = 0.0f;
	int boid_num = 0;

	// Evaluate the performance of each flock member
	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// Weight values as seen to be appropriate:
		float fitfunc = iterator->GetHealth() + flock_health - enemy_flock_health;
		iterator->SetFitness(fitfunc);

		sum_fitness_ += fitfunc;
		boid_num++;

		if (highest_fitness_ < fitfunc) // Calculate the highest fitness level
		{
			highest_fitness_ = fitfunc;
		}
		else if (lowest_fitness_ > fitfunc) // Calculate the lowest fitness level
		{
			lowest_fitness_ = fitfunc;
		}
	}
	// Get the value for the mean fitness:
	mean_fitness_ = sum_fitness_ / (float)boid_num;
	// Get the value for the fitness range
	range_fitness_ = highest_fitness_ - lowest_fitness_;
}

void genetic_algorithm::Selection(std::vector<boid>* boids)
{
	// Empty the list to be refilled for this generation:
	top_boid_strains_.clear();

	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// This will allow some weaker strains to pass through the filter:
		float selection_probability = 0.001f * (rand() % (int)((range_fitness_ + lowest_fitness_)*1000.0f));

		if (iterator->GetFitness() > selection_probability || iterator->GetFitness() >= mean_fitness_)
		{
			// Add the iterator to the list of top boids
			top_boid_strains_.push_back(*iterator);
		}
	}
}

void genetic_algorithm::Crossover()
{
	// Not like this, this should work as pairs of parents tbh that then produce offspring
	for (std::vector<boid>::iterator iterator = top_boid_strains_.begin(); iterator != top_boid_strains_.end(); iterator++)
	{

	}
}

void genetic_algorithm::Mutation(std::vector<boid>* boids, int generation)
{
	int file_num = 0;

	// For each boid:
	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// populate the genetic data
		DNA dna = iterator->GetDNA();

		float data[12];
		float genetic_variation;

		for (int point = 0; point < 12; point++)
		{
			// Probability of Mutation
			// rand (15% chance of mutation for this variable)

			// Modification based off of the percentage, so all modifications are done in the same relative scale

			data[point] = dna.GetData(point);// + mutating factor[point];
		}

		dna.UpdateDataSet(data);

		std::string txt_directory("GeneticDatatxt/Generation_");
		std::string txt_file_name("/boid_data_");
		std::string txt_base(".txt");

		std::string csv_directory("GeneticDataCSV/Simulation_");
		std::string csv_file_name("/boid_data_");
		std::string csv_base(".csv");

		dna.StoreData(
			txt_directory + std::to_string(generation) + txt_file_name + std::to_string(file_num) + txt_base,
			csv_directory + std::to_string(glo_simulation_number) + csv_file_name + std::to_string(file_num) + csv_base);
		file_num++;
		// Add the newly produced dna to the boid:
		iterator->SetDNA(dna);
	}
}

void genetic_algorithm::GeneticSnapshot()
{

}
