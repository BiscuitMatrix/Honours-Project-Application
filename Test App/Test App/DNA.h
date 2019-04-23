#pragma once

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>


class DNA
{
public:
	DNA();
	~DNA();
	void CleanUp();

	void StoreData(int, std::string, std::string);
	void ReadData(std::string);

	float GetData(int access_point) { return data_[access_point]; }
	void SetData(int access_point, float val) { data_[access_point] = val; }
	void UpdateDataPoint(int index, float data_point) { data_[index] = data_point; }
	void UpdateDataSet(float data[]) 
	{ 
		for (int i = 0; i < 12; i++)
		{
			data_[i] = data[i];
		}
	};

	float GetCohMult() { return *coh_mult_; }
	float GetCohDivMult() { return *coh_div_mult_; }
	float GetAliMult() { return *ali_mult_; }
	float GetAliDivMult() { return *ali_div_mult_; }
	float GetSepMult() { return *sep_mult_; }
	float GetSepNeighMult() { return *sep_neigh_mult_; }
	float GetSepCloNeighMult() { return *sep_clo_neigh_mult_; }
	float GetFoodMult1() { return *food_mult_1_; }
	float GetFoodMult2() { return *food_mult_2_; }
	float GetFoodDivMult() { return *food_div_mult_; }
	float GetFloAvMult() { return *floav_mult_; }
	float GetFloAvDivMult() { return *floav_div_mult_; }


private:
	// Data will be in this order in the text files!
	// Cohesion
	float* coh_mult_;				// data[0]
	float* coh_div_mult_;			// data[1]
	//Alignment
	float* ali_mult_;				// data[2]
	float* ali_div_mult_;			// data[3]
	// Separation
	float* sep_mult_;				// data[4]
	float* sep_neigh_mult_;			// data[5]
	float* sep_clo_neigh_mult_;		// data[6]
	// Food Attraction
	float* food_mult_1_;			// data[7]
	float* food_mult_2_;			// data[8]
	float* food_div_mult_;			// data[9]
	// Flock Avoidance
	float* floav_mult_;				// data[10]
	float* floav_div_mult_;			// data[11]

	// Maybe a more concise version:
	float data_[12];
};

