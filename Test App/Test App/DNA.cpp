#include "DNA.h"

DNA::DNA() :
	coh_mult_			(&data_[0]),
	coh_div_mult_		(&data_[1]),
	ali_mult_			(&data_[2]),
	ali_div_mult_		(&data_[3]),
	sep_mult_			(&data_[4]),
	sep_neigh_mult_		(&data_[5]),
	sep_clo_neigh_mult_	(&data_[6]),
	food_mult_1_		(&data_[7]),
	food_mult_2_		(&data_[8]),
	food_div_mult_		(&data_[9]),
	floav_mult_			(&data_[10]),
	floav_div_mult_		(&data_[11])
{
}

DNA::~DNA()
{
}
void DNA::CleanUp()
{
	coh_mult_ = nullptr;
	coh_div_mult_ = nullptr;
	ali_mult_ = nullptr;
	ali_div_mult_ = nullptr;
	sep_mult_ = nullptr;
	sep_neigh_mult_ = nullptr;
	sep_clo_neigh_mult_ = nullptr;
	food_mult_1_ = nullptr;
	food_mult_2_ = nullptr;
	food_div_mult_ = nullptr;
	floav_mult_ = nullptr;
	floav_div_mult_ = nullptr;
}

void DNA::StoreData(std::string txt_file, std::string csv_data)
{
	//if ((txt_file.substr(txt_file.find_last_of(".") + 1) != ".txt") 
	//	|| (csv_data.substr(csv_data.find_last_of(".") + 1) != ".csv"))
	//{
	//	float whyohwhy = 0;
	//}

	std::ofstream stored_data;
	stored_data.open(txt_file);
	for (int i = 0; i < 12; i++)
	{
		stored_data << data_[i] << "\n";
	}
	stored_data.close();

	std::ofstream excel_data;
	excel_data.open(csv_data);
	for (int i = 0; i < 12; i++)
	{
		excel_data << data_[i] << ",\n";
	}
	excel_data.close();
}

void DNA::ReadData(std::string data_file)
{
	std::ifstream stored_data_;
	stored_data_.open(data_file);

	if (stored_data_.is_open())
	{
		for (int i = 0; i < 12; i++)
		{
			stored_data_ >> data_[i];
		}
	}

	// Debugging
	float a, b, c, d, e, f, g, h, i, j, k, l, m;
	a = data_[0];
	b = data_[1];
	c = data_[2];
	d = data_[3];
	e = data_[4];
	f = data_[5];
	g = data_[6];
	h = data_[7];
	i = data_[8];
	j = data_[9];
	k = data_[10];
	l = data_[11];

	m = 1337;
}


