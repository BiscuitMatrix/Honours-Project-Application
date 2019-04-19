#include "genetic_algorithm.h"



genetic_algorithm::genetic_algorithm() :
	generation_(0),
	population_(glo_flock_size)
{
}


genetic_algorithm::~genetic_algorithm()
{
}

void genetic_algorithm::CleanUp()
{

}


void genetic_algorithm::Initialise(std::vector<boid> *boids)
{
	int i = 0;
	// For each boid:
	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// populate the genetic data
			DNA dna = DNA();

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

			dna.UpdateDataSet(data);

			std::string txt_folder("GeneticDatatxt/Generation_1/boid_data_");
			std::string txt_base(".txt");

			std::string csv_folder("GeneticDataCSV/Simulation_1/boid_data_");
			std::string csv_base(".csv");

			dna.StoreData(txt_folder+std::to_string(i)+txt_base, csv_folder+std::to_string(i)+csv_base);
			i++;
			//genetic_information_.push_back(dna);

			iterator->SetDNA(dna);
	}
}

void genetic_algorithm::Update(std::vector<boid>* boids)
{
	int i = 0;
	// For each boid:
	for (std::vector<boid>::iterator iterator = boids->begin(); iterator != boids->end(); iterator++)
	{
		// populate the genetic data
		DNA dna = iterator->GetDNA();

		float data[12];
		float genetic_variation;

		for (int j = 0; j < 12; j++)
		{
			// Probability of Mutation
			// rand (15% chance of mutation for this variable)

			// Modification based off of the percentage, so all modifications are done in the same relative scale

			data[j] = dna.GetData(j);// + mutating factor[j];
		}

		dna.UpdateDataSet(data);

		std::string txt_folder("GeneticDatatxt/Generation_1/boid_data_");
		std::string txt_base(".txt");

		std::string csv_folder("GeneticDataCSV/Simulation_1/boid_data_");
		std::string csv_base(".csv");

		dna.StoreData(txt_folder + std::to_string(i) + txt_base, csv_folder + std::to_string(i) + csv_base);
		i++;
		//genetic_information_.push_back(dna);

		iterator->SetDNA(dna);
	}
}

void genetic_algorithm::Evaluate()
{

}

void genetic_algorithm::Selection()
{

}

void genetic_algorithm::Crossover()
{

}

void genetic_algorithm::GeneticSnapshot()
{

}
