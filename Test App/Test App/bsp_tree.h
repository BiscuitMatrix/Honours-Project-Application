#pragma once

struct node
{
	int* data_;

	node* prev_;
	node* left_;
	node* right_;
};

class bsp_tree
{
public:
	bsp_tree();
	~bsp_tree();
};

