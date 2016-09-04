#ifndef KDTREE
#define KDTREE

#include<bits\stdc++.h>
#include "RRT.h"
#define vi vector<int>
using namespace std;

class NNret {
public:
	int x, y;
	long double d;
	node * nde;
	NNret(int &x, int &y, long double d, node *nde);
	NNret();

};

class KDTree {
public:
	int x, y;
	int dim, axis;
	KDTree *left, *right;
	node * nde;
	KDTree(int &x, int &y, int axis, KDTree* left, KDTree* right, node* nde);
	void insert(int &x, int &y, node *nde);
	long double dist(int &x, int &y);
	void printTree(void);
	NNret search(int &x, int &y, long double dist, int &refx, int &refy, node* refnde);
	void searchRNN(int &x, int &y, long double dist, vector<node* > &v);
	KDTree* deletePoint(int &x, int &y);
	bool ifPresent(int&x, int&y);
	
};

KDTree* deleteRec(KDTree* root, int &x, int &y);
KDTree* findMin(KDTree* root, int &d);
KDTree* minNode(KDTree* x, KDTree* y, KDTree* z, int &depth);
KDTree* deleteEverything(KDTree* Point);
#endif