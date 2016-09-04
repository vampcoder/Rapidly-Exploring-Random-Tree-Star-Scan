#ifndef RRTHeader
#define RRTHeader

#include<bits\stdc++.h>
#define vi vector<int>

using namespace std;

class node {
public:
	int x, y;
	long double cost;
	node *parent;
	vector<node *> children;

	node(int &x, int &y, long double cost, node* parent, vector<node* > &children);
	void add_child(node* node1);
	void propogateCost(void);
};
#endif // !RRTHeader
