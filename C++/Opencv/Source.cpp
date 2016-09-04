#include "KdTree.h"


int main() {
	int n, d, x;
	cin >> n >> d;
	KDTree *tree = NULL;
	for (int i = 0; i < n; i++) {
		vector<int> pnt;
		for (int j = 0; j < d; j++) {
			cin >> x;
			pnt.push_back(x);
		}
		if (tree == NULL) {
			tree = new KDTree(pnt, d, 0, NULL, NULL);
		}
		else {
			tree->insert(pnt);
		}
	}
	tree->printTree();
}
