#include "KdTree.h"

using namespace std;


NNret::NNret(int &x, int &y, long double d, node *nde) {
	this->x = x;
	this->y = y;
	this->d = d;
	this->nde = nde;
}

NNret::NNret() {
	;
}

KDTree::KDTree(int &x, int &y, int axis, KDTree* left, KDTree* right, node* nde) {
		this->x = x;
		this->y = y;
		this->axis = axis;
		this->left = left;
		this->right = right;
		this->nde = nde;
	}

void KDTree::insert(int &x, int &y, node *nde) {
	int x1 = this->x;
	int y1 = this->y;
		int axis = this->axis;

		if (axis == 0) {
			if (x1 > x) {
				if (this->left == NULL) {
					this->left = new KDTree(x, y, (axis + 1) % 2, NULL, NULL, nde);
				}
				else {
					this->left->insert(x, y, nde);
				}
			}
			else {
				if (this->right == NULL) {
					this->right = new KDTree(x, y ,(axis + 1) % 2, NULL, NULL, nde);
				}
				else {
					this->right->insert(x, y, nde);
				}
			}
		}
		else {
			if (y1 > y) {
				if (this->left == NULL) {
					this->left = new KDTree(x, y, (axis + 1) % 2, NULL, NULL, nde);
				}
				else {
					this->left->insert(x, y, nde);
				}
			}
			else {
				if (this->right == NULL) {
					this->right = new KDTree(x, y, (axis + 1) % 2, NULL, NULL, nde);
				}
				else {
					this->right->insert(x, y, nde);
				}
			}
		}
	}

void KDTree::printTree(void) {
	if (this->left != NULL) {
		this->left->printTree();
	}
	cout << this->x << " " << this->y << endl;
	if (this->right != NULL) {
		this->right->printTree();
	}
}

long double KDTree::dist(int &x, int &y) {
	long double ans = 0;
	ans = sqrt((this->x -x)*(this->x-x)*1.0 + (this->y-y)*(this->y-y)*1.0);
	return ans;
}

NNret KDTree::search(int &x, int &y,long double dist, int &refx, int &refy, node *refnde) {
	int axis = this->axis;
	if (this->left == NULL && this->right == NULL) 
	{
		long double w = this->dist(x, y);
		if (w < dist) {
			NNret ret = NNret(this->x, this->y, w, this->nde);
			return ret;
		}
		else {
			NNret ret = NNret(refx, refy, dist, refnde);
			return ret;
		}
	}
	else {
		long double d = this->dist(x, y);
		if (d < dist) {
			dist = d;
			refx = this->x;
			refy = this->y;
			refnde = this->nde;
		}
		if (axis == 0) {
			if (x <= this->x) {
				if (x - dist <= this->x) {
					if (this->left != NULL) {
						NNret ret = this->left->search(x, y, dist, refx, refy, refnde);
						refx = ret.x;
						refy = ret.y;
						dist = ret.d;
						refnde = ret.nde;
					}
				}
				if (x + dist > this->x) {
					if (this->right != NULL) {
						return this->right->search(x, y, dist, refx, refy, refnde);
					}
				}
			}
			else {
				if (x + dist > this->x) {
					if (this->right != NULL) {
						NNret ret = this->right->search(x, y, dist, refx, refy, refnde);
						dist = ret.d;
						refx = ret.x;
						refy = ret.y;
						refnde = ret.nde;
					}
				}
				if (x - dist <= this->x) {
					if (this->left != NULL) {
						return this->left->search(x, y, dist, refx, refy, refnde);
					}
				}
			}
		}
		else {
			if (y <= this->y) {
				if (y - dist <= this->y) {
					if (this->left != NULL) {
						NNret ret = this->left->search(x, y, dist, refx, refy, refnde);
						refx = ret.x;
						refy = ret.y;
						dist = ret.d;
						refnde = ret.nde;
					}
				}
				if (y + dist > this->y) {
					if (this->right != NULL) {
						return this->right->search(x, y, dist, refx, refy, refnde);
					}
				}
			}
			else {
				if (y + dist > this->y) {
					if (this->right != NULL) {
						NNret ret = this->right->search(x, y, dist, refx, refy, refnde);
						dist = ret.d;
						refx = ret.x;
						refy = ret.y;
						refnde = ret.nde;
					}
				}
				if (y - dist <= this->y) {
					if (this->left != NULL) {
						return this->left->search(x, y, dist, refx, refy, refnde);
					}
				}
			}

		}
		return NNret(refx, refy, dist, refnde);
	}
}

void KDTree::searchRNN(int &x, int &y, long double d, vector<node* > &v) {
	int axis = this->axis;
	if (this->left == NULL && this->right == NULL) {
		long double w = this->dist(x, y);
		if (w < d) {
			v.push_back(this->nde);
		}
		return;
	}
	else {
		long double w = this->dist(x, y);
		if (w < d) {
			v.push_back(this->nde);
		}
		if (axis == 0) {
			if (x < this->x) {
				if (x - d <= this->x && this->left != NULL) {
					this->left->searchRNN(x, y, d, v);
				}
				if (x + d > this->x && this->right != NULL) {
					this->right->searchRNN(x, y, d, v);
					return;
				}
			}
			else {
				if (x + d > this->x && this->right != NULL) {
					this->right->searchRNN(x, y, d, v);
				}
				if (x - d <= this->x && this->left != NULL) {
					this->left->searchRNN(x, y, d, v);
					return;
				}
			}
		}else{
			if (y < this->y) {
				if (y - d <= this->y && this->left != NULL) {
					this->left->searchRNN(x, y, d, v);
				}
				if (y + d > this->y && this->right != NULL) {
					this->right->searchRNN(x, y, d, v);
					return;
				}
			}
			else {
				if (y + d > this->y && this->right != NULL) {
					this->right->searchRNN(x, y, d, v);
				}
				if (y - d <= this->y && this->left != NULL) {
					this->left->searchRNN(x, y, d, v);
					return;
				}
			}
		}
	}
}

KDTree* minNode(KDTree* x, KDTree* y, KDTree* z, int &depth) {
	KDTree* res = x;
	if (depth == 0) {
		if (y != NULL && y->x < res->x)
			res = y;
		if (z != NULL && z->x < res->x)
			res = z;
		return res;
	}
	else {
		if (y != NULL && y->y < res->y)
			res = y;
		if (z != NULL && z->y < res->y)
			res = z;
		return res;
	}
}

KDTree* findMin(KDTree* root, int &d) {
	if (root == NULL)
		return root;
	int cd = root->axis;
	if (cd == d) {
		if (root->left == NULL) {
			return root;
		}
		return findMin(root->left, d);
	}

	return minNode(root, findMin(root->left, d), findMin(root->right, d), d);
}

KDTree* deleteRec(KDTree* root, int &x, int &y) {
	if (root == NULL) {
		return NULL;
	}
	int cd = root->axis;
	if (root->x == x and root->y == y) {
		if (root->right != NULL) {
			KDTree* nde = findMin(root->right, cd);
			root->x = nde->x;
			root->y = nde->y;
			root->right = deleteRec(root->right, nde->x, nde->y);
		}
		else if (root->left != NULL) {
			KDTree* nde = findMin(root->left, cd);
			root->x = nde->x;
			root->y = nde->y;
			root->right = deleteRec(root->left, nde->x, nde->y);
			root->left = NULL;
		}
		else {
			delete root;
			return NULL;
		}
		return root;
	}
	if (cd == 0) {
		if (x < root->x) {
			root->left = deleteRec(root->left, x, y);
		}
		else {
			root->right = deleteRec(root->right, x, y);
		}
	}
	else {
		if (y < root->y) {
			root->left = deleteRec(root->left, x, y);
		}
		else {
			root->right = deleteRec(root->right, x, y);
		}
	}
	return root;
}

KDTree* KDTree::deletePoint(int &x, int &y) {
	/*int refx = -100000000;
	int refy = -100000000;
	NNret ret = this->search(x, y, LDBL_MAX, refx, refy, NULL);
	if (ret.d >= 1) {
		return this;
	}*/
	return deleteRec(this, x, y);

}

bool KDTree::ifPresent(int& x, int& y) {
	int axix = this->axis;
	if (this->x == x && this->y == y) {
		return true;
	}
	if (this->left == NULL && this->right == NULL)
		return false;
	bool ans = false;
	if (axis == 0) {
		if (this->x > x ) {
			if(this->left != NULL)
				ans = ans || this->left->ifPresent(x, y);
		}
		else {
			if (this->right != NULL)
				ans = ans || this->right->ifPresent(x, y);
		}
	}
	else {
		if (this->y > y) {
			if (this->left != NULL)
				ans = ans || this->left->ifPresent(x, y);
		}
		else {
			if (this->right != NULL)
				ans = ans ||this->right->ifPresent(x, y);
		}
	}
	return ans;
}

KDTree* deleteEverything(KDTree* Point) {
	if (Point->left != NULL) {
		Point->left = deleteEverything(Point->left);
	}
	if (Point->right != NULL) {
		Point->right = deleteEverything(Point->right);
	}
	delete(Point);
	return NULL;
}