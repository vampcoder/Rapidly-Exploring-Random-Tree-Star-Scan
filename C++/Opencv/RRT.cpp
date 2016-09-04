#include "RRT.h"

using namespace std;

node::node(int &x, int &y, long double cost, node* parent, vector<node* > &children) {
	this->x = x;
	this->y = y;
	this->cost = cost;
	this->parent = parent;
	this->children = children;
}

void node::add_child(node* node1) {
	this->children.push_back(node1);
}

long double dist(int &x1, int &y1, int &x2, int &y2) {
	return sqrt((x1-x2)*(x1-x2)*1.0 + (y1-y2)*(y1-y2)*1.0);
}

void node::propogateCost(void) {
	queue<node *> q;
	q.push(this);
	node * cur= NULL;
	while (!q.empty()) {
		cur = q.front();
		q.pop();
		for (int i = 0; i < cur->children.size(); i++) {
			//if (cur->children[i]->cost > cur->cost + dist(cur->x, cur->y, cur->children[i]->x, cur->children[i]->y)) {
				cur->children[i]->cost = cur->cost + dist(cur->x, cur->y, cur->children[i]->x, cur->children[i]->y);
				q.push(cur->children[i]);
			//}
		}
	}
}