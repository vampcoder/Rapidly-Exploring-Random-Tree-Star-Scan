#include "RRT-star-Implementation.h"

bool flag = false;
int sx1 = 55, sy1 = 530;
int dx1 = 1082, dy1 = 33;
bool show = true;

void showImage(string winName, Mat &image) {
	if (show) {
		imshow(winName, image);
		int k = waitKey(1);
		if (k == 27) {
			exit(0);
		}
	}
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) {
		if (flag) {
			dx1 = x;
			dy1 = y;
			cout << x << " " << y << endl;
		}
		else {
			flag = true;
			sx1 = x;
			sy1 = y;
			cout << x << " " << y << endl;
		}
	}
}

void func(vector<node*> &v, node* n, vector<node*>::iterator &it) {
	it = v.begin();
	node* temp;
	while (it != v.end()) {
		temp = *it;
		if (temp->x == n->x && temp->y == n->y) {
			return;
		}
		it++;
	}
}

workspace::workspace() {
	this->image = imread("Images/obstacle.png");
	if (this->image.empty()) {
		cout << "Can't load the image" << endl;
		return;
	}
	Mat image;
	//Mat image = Mat(this->image.rows, this->image.cols, CV_8UC3, Scalar(0, 0, 0));
	cvtColor(this->image, image, CV_BGR2GRAY);
	//this->treeimage = image.clone();
	this->image = image;
	this->sx = -1;
	this->sy = -1;
	this->dx = -1;
	this->dy = -1;
	this->RRTree = NULL;
	this->Points = NULL;
	this->TempPoints = NULL;
	this->goalFound = false;
	this->tempGoalFound = false;
	this->goalNode = NULL;
}

void workspace::getSourceandGoal() {
	namedWindow("image", CV_WINDOW_AUTOSIZE);
	
	setMouseCallback("image", CallBackFunc, NULL);
	imshow("image", this->image);
	waitKey(0);
	destroyAllWindows();
	this->sx = sx1;
	this->sy = sy1;
	
	this->dx = dx1;
	this->dy = dy1;
	this->cx = sx;
	this->cy = sy;
	cout << this->image.rows << " " << this->image.cols << endl;
}

//Returns True if goal found. 
bool workspace::checkifgoalFound(int &x, int &y) {
	if ((x >= this->dx - 2) && (x <= this->dx + 2) && (y >= this->dy - 2) && (y <= this->dy + 2)) {
		return true;
	}
	else {
		return false;
	}
}

//Returns True, if inside boundaries
bool workspace::checkBoundaries(int &x, int &y) {
	int rows = this->image.rows;
	int cols = this->image.cols;
	if ((x < 0) || (x >= cols) || (y < 0) || (y >= rows)) {
		return false;
	}
	else {
		return true;
	}
}

bool workspace::checkWithinRadius(int &x, int &y) {
	if (this->dist(this->cx, this->cy, x, y) <= RADIUS) {
		return true;
	}
	return false;
}
//Generating Random point
void workspace::generatePoint(int &x, int &y) {
	long double p = rand() % 100;
	int cols = this->image.cols;
	int rows = this->image.rows;
	if (p < 5) {
		x = this->dx;
		y = this->dy;
		return;
	}
	else if (p < 10) {
		do {
			x = rand() % (cols);
			y = rand() % (rows);
		} while (!this->checkWithinRadius(x, y));
	}
	else if (p < 70) {
		do {
			x = rand() % (cols);
			y = rand() % (rows);
		} while (!this->obstacleFreeEdge(x, y, this->cx, this->cy));
	}
	else {
		x = rand() % (cols);
		y = rand() % (rows);

	}
}
// Return true if point is in obstacle free region
bool workspace::obstacleFree(int &x, int &y) {

	Scalar intensity = this->visibleimage.at<uchar>(y, x);
	if (this->checkBoundaries(x, y)) {
		//cout << "intensity " << intensity.val[0] << endl;
		if (intensity.val[0] == 255 || intensity.val[0] == 0) {
			return false;
		}
		else {
			return true;
		}
	}
	return false;
}
//Returns distance between given points
long double workspace::dist(int &x1, int &y1, int &x2, int &y2) {
	return sqrt((x1 - x2)*(x1 - x2)*1.0 + (y1 - y2)*(y1 - y2)*1.0);
}
//Find point using steering function
void workspace::stepto(int &x1, int &y1, int &x2, int &y2) {
	if (this->dist(x1, y1, x2, y2) < 1.0*EPSILON) {
		return;
	}
	double theta = atan2(y2-y1, x2-x1);
	vector<int> ans;
	x2 = (int(x1 + EPSILON* cos(theta)));
	y2 = (int(y1 + EPSILON* sin(theta)));
}
//check if points are same
bool workspace::check_same(int &x1, int &y1, int &x2, int &y2) {
	if ((x1 <= x2 + 1) && (x1 >= x2 - 1) && (y1 <= y2 + 1) && (y1 >= y2 - 1)) {
		return true;
	}
	return false;
}
//Check if edge is obstacle free
bool workspace::obstacleFreeEdge(int &x1, int &y1, int &x2, int &y2) {
	double theta = atan2(y2-y1, x2-x1);
	if (!this->obstacleFree(x1, y1) || !this->obstacleFree(x2, y2))
		return false;
	int x = x1;
	int y = y1;
	int i = 0;
	while (!check_same(x2, y2, x, y)) {
		x = int(x1 + i*cos(theta));
		y = int(y1 + i*sin(theta));
		if (!this->checkBoundaries(x, y)) {
			//cout << "true" << endl;
			return false;
		}
		if (!this->obstacleFree(x, y))
			return false;
		i++;
	}
	return true;
}
//check if point is in black region
bool workspace::check_for_black(int &x1, int &y1, int &x2, int &y2) {
	return not this->obstacleFreeEdge(x1, y1, x2, y2);
}
//check if point is in gray region
bool workspace::check_for_gray(int &x, int &y) {
	Scalar intensity = this->visibleimage.at<uchar>(y, x);
	return intensity.val[0] == 0;
}
//It generates random points with some random biasness
void workspace::generateGoalBiasedPoint(int &x, int &y) {
	long double p = rand() % 100;
	if (p < 70 && !this->tempGoalFound) {
		x = this->dx;
		y = this->dy;
		return;
	}
	int cols = this->image.cols;
	int rows = this->image.rows;
	x = rand() % (cols);
	y = rand() % (rows);
}
//Check if Points are temporary of not
bool workspace::checkForTempPoints(int &x, int &y) {
	Scalar Intensity = this->visibleimage.at<uchar>(y, x);
	return Intensity.val[0] != 0;
}
//Update new root after iteration
void workspace::updateRoot(node* root) {
	vector<node*>::iterator it;
	func(this->RRTree->children, root, it);
	this->RRTree->children.erase(it);
	root->children.push_back(this->RRTree);
	this->RRTree->parent = root;
	root->parent = NULL;
	this->RRTree = root;
	this->RRTree->cost = 0;
	this->RRTree->propogateCost();
}
//check if obstacle free in goal biased black tree
bool workspace::obstacleFreeGoalBiased(int &x, int &y) {
	Scalar intensity = this->visibleimage.at<uchar>(y, x);
	if (this->checkBoundaries(x, y)) {
		//cout << "intensity " << intensity.val[0] << endl;
		if (intensity.val[0] == 255) {
			return false;
		}
		else {
			return true;
		}
	}
	return false;
}
//check if obstacle free edge in goal biased black tree
bool workspace::obstacleFreeEdgeGoalBiased(int &x1, int &y1, int &x2, int &y2) {
	double theta = atan2(y2 - y1, x2 - x1);
	if (!this->obstacleFreeGoalBiased(x1, y1) || !this->obstacleFreeGoalBiased(x2, y2))
		return false;
	int x = x1;
	int y = y1;
	int i = 0;
	while (!check_same(x2, y2, x, y)) {
		x = int(x1 + i*cos(theta));
		y = int(y1 + i*sin(theta));
		if (!this->checkBoundaries(x, y)) {
			//cout << "true" << endl;
			return false;
		}
		if (!this->obstacleFreeGoalBiased(x, y))
			return false;
		i++;
	}
	return true;
}
//take next step after iteration
void workspace::takeNextStep() {
	int refx = -10000000000;
	int refy = -10000000000;
	
	//NNret ret = this->TempPoints->search(this->dx, this->dy, LDBL_MAX, refx, refy, NULL);
	node *nde = this->tempGoalNode;
	if (nde == NULL) {
		cout << "NULL in Take Next Step" << endl;
	}
	while (nde->parent->parent != NULL) {
		nde = nde->parent;
	}
	this->cx = nde->x;
	this->cy = nde->y;
	this->updateRoot(nde);
	this->steps.push_back(make_pair(this->cx, this->cy));
}
//Generate GoalBiasedBlackRRT
void workspace::GoalBiasedBlackRRT() {
	int i = 0;
	while (true) {
		int x, y, x1, y1;
		int refx = -100000;
		int refy = -100000;
		do {
			this->generateGoalBiasedPoint(x, y);
		} while (this->checkForTempPoints(x, y));
		NNret ret = this->Points->search(x, y, LDBL_MAX, refx, refy, NULL);
		NNret ret1 = ret;
		if (this->TempPoints != NULL) {
			refx = refy = -1000000;
			ret1 = this->TempPoints->search(x, y, LDBL_MAX, refx, refy, NULL);
		}
		if (ret.d > ret1.d) {
			ret = ret1;
		}
		x1 = ret.x;
		y1 = ret.y;
		this->stepto(x1, y1, x, y);
		if (this->TempPoints != NULL) {
			if (this->TempPoints->ifPresent(x, y)) {
				continue;
			}
		}
		if (this->obstacleFreeEdgeGoalBiased(x, y, x1, y1)) {
			i++;
			vector<node* > v;
			node *nde = new node(x, y, ret.nde->cost + this->dist(x, y, x1, y1), ret.nde, v);
			ret.nde->add_child(nde);
			this->tempPointsarr.push_back(nde);
			if (this->TempPoints == NULL) {
				this->TempPoints = new KDTree(x, y, 0, NULL, NULL, nde);
			}
			else {
				this->TempPoints->insert(x, y, nde);
			}
			if (this->checkifgoalFound(x, y)) {
				this->tempGoalFound = true;
				this->tempGoalNode = nde;
				this->takeNextStep();
				break;
			}
			cv::line(this->treeimage, Point(x, y), Point(x1, y1), Scalar(150));
			showImage("treeimage", this->treeimage);
		}
	}
	/*if (i == 200)
		this->tempGoalFound = false;*/
}
//Removing leaf nodes from original tree
void workspace::removeGeneratedLeafNodes() {
	for (int i = 0; i < this->tempPointsarr.size(); i++) {
		node* nde = tempPointsarr[i];
		node* parent = nde->parent;
		vector<node*>::iterator it;
		func(parent->children, nde, it);
		parent->children.erase(it);
		nde->parent = NULL;
		this->TempPoints = this->TempPoints->deletePoint(nde->x, nde->y);
	}
	//this->TempPoints->printTree();
	//cout << endl;
	for (int i = 0; i < this->tempPointsarr.size(); i++) {
		delete(tempPointsarr[i]);
		tempPointsarr[i] = NULL;

	}
	/*if (this->TempPoints == NULL)
		cout << "Null" << endl;*/
	/*else {
		this->TempPoints->printTree();
	}*/
	this->tempGoalFound = false;
	this->tempGoalNode = NULL;
	this->tempPointsarr.clear();
}

//Simple GoalBiased RRT for Black region
void workspace::type1() {
	this->GoalBiasedBlackRRT();
	this->removeGeneratedLeafNodes();
}

void workspace::removeConnectionOfGeneratedNodes() {
	for (int i = 0; i < this->tempPointsarr.size(); i++) {
		node* nde = this->tempPointsarr[i];
		node* parent = nde->parent;
		if (this->Points->ifPresent(parent->x, parent->y)) {
			this->archievedTree.push_back(nde);
			vector<node*>::iterator it;
			func(parent->children, nde, it);
			parent->children.erase(it);
		}
		//this->TempPoints = this->TempPoints->deletePoint(nde->x, nde->y);
	}
	this->tempPointsarr.clear();
	this->tempGoalFound = false;
}

void workspace::deleteBlackSubtree(node* nde) {

	for (int i = 0; i < nde->children.size(); i++) {
		this->deleteBlackSubtree(nde->children[i]);
	}
	if (this->TempPoints->ifPresent(nde->x, nde->y)) {
		this->TempPoints = this->TempPoints->deletePoint(nde->x, nde->y);
		//cout << "deleted " << this->TempPoints->ifPresent(nde->x, nde->y) << endl;;
	}
	nde->children.clear();
	nde->parent = NULL;
	node* nde1 = nde;
	nde = NULL;
	delete nde1;
}

void workspace::printBlackSubtree(node *nde) {
	if (this->obstacleFreeEdgeGoalBiased(nde->x, nde->y, nde->parent->x, nde->parent->y)) {
		cv::line(this->treeimage, Point(nde->x, nde->y), Point(nde->parent->x, nde->parent->y), Scalar(150));

		showImage("treeimage", this->treeimage);

		if (!this->tempGoalFound && this->checkifgoalFound(nde->x, nde->y)) {
			this->tempGoalFound = true;
			this->tempGoalNode = nde;
		}
		this->tempPointsarr.push_back(nde);
		for (int i = 0; i < nde->children.size(); i++) {
			node* nde1 = nde->children[i];
			this->printBlackSubtree(nde1);
		}
	}
	else {
		//cout << "yes3" << endl;
		vector<node* >::iterator it;
		func(nde->parent->children, nde, it);
		nde->parent->children.erase(it);
		//cout << "yes2" << endl;
		this->deleteBlackSubtree(nde);
	}
}

void workspace::useArchieveTree() {
	for (int i = 0; i < this->archievedTree.size(); i++) {
		node* nde = this->archievedTree[i];
		node* parent = nde->parent;
		if (this->nonObstacleFree(nde->x, nde->y) && this->obstacleFreeEdgeGoalBiased(nde->x, nde->y, parent->x, parent->y)) {
			nde->parent->add_child(nde);
			//cout << "yes" << endl;
			this->printBlackSubtree(nde);
			//cout << "no" << endl;
		}
		else {
			this->deleteBlackSubtree(nde);
		}
	}
	this->archievedTree.clear();
}

//Simple GoalBiased RRT for black region with memory(Archive tree concept)
void workspace::type2() {
	this->useArchieveTree();
	if (this->tempGoalFound) {
		//cout << "DirectlyNextStep" << endl;
		this->takeNextStep();
	}
	else {
		//cout << "goalBiasedBlackRRT" << endl;
		this->GoalBiasedBlackRRT();
	}
	showImage("treeimage", this->treeimage);
	
	this->removeConnectionOfGeneratedNodes();

}

//RRT-star in black region
void workspace::type3() {


}

bool workspace::nonObstacleFree(int &x, int &y) {
	if (!this->checkBoundaries(x, y))
		return false;
	Scalar intensity = this->visibleimage.at<uchar>(y, x);
		if (intensity.val[0] != 0) {
			return false;
		}
		else {
			return true;
		}
}

bool workspace::obstacleEdge(int &x1, int &y1, int &x2, int &y2) {
	double theta = atan2(y2 - y1, x2 - x1);
	if (!this->nonObstacleFree(x1, y1) || !this->nonObstacleFree(x2, y2))
		return false;
	int x = x1;
	int y = y1;
	int i = 0;
	while (!check_same(x2, y2, x, y)) {
		x = int(x1 + i*cos(theta));
		y = int(y1 + i*sin(theta));
		if (!this->checkBoundaries(x, y)) {
			return false;
		}
		if (!this->nonObstacleFree(x, y))
			return false;
		i++;
	}
	return true;
}

void workspace::generateGoalRootedPoint(int &x, int &y) {
	long double p = rand() % 100;
	if (p < 30) {
		int idx = (int)((rand() % 100) * this->points.size())/100;
		if (idx < 0) {
			idx = 0;
		}
		if (idx >= this->points.size())
			idx = this->points.size() - 1;
		x = this->points[idx].first;
		y = this->points[idx].second;
		return;
	}
	int cols = this->image.cols;
	int rows = this->image.rows;
	x = rand() % (cols);
	y = rand() % (rows);
}

void workspace::createLinkGoalRooted(node *childlink, node * parentlink) {
	node *oldparent, *child, *parent;
	child = childlink;
	parent = parentlink;
	vector<node*>::iterator it;
	cv::line(this->treeimage, Point(childlink->x, childlink->y), Point(parent->x, parent->y), Scalar(200));

	while (true) {
		oldparent = child->parent;
		func(oldparent->children, child, it);
		oldparent->children.erase(it);
		child->parent = parent;
		parent->children.push_back(child);
		if ((oldparent->cost > child->cost + this->dist(child->x, child->y, oldparent->x, oldparent->y))) {
			oldparent->cost = child->cost + this->dist(child->x, child->y, oldparent->x, oldparent->y);
			parent = child;
			child = oldparent;
		}
		else {
			cv::line(this->treeimage, Point(child->x, child->y), Point(oldparent->x, oldparent->y), Scalar(0));
			return;
		}
	}
}

void workspace::addConnectionGoalRooted(int &x1, int &y1) {
	int x, y;
	this->generateGoalRootedPoint(x, y);
	NNret NN;
	
	while (true) {

		int refx = 10000000;
		int refy = 10000000;
		NN = this->TempPoints->search(x, y, LDBL_MAX, refx, refy, NULL);
		x1 = NN.x;
		y1 = NN.y;

		this->stepto(x1, y1, x, y);
		if (!this->TempPoints->ifPresent(x, y)) {
			if (this->obstacleEdge(x, y, x1, y1)) {
				break;
			}
		}

		this->generateGoalRootedPoint(x, y);
	}

	vector<node* > v1;
	this->TempPoints->searchRNN(x, y, RADIUS, v1);

	node* nde1 = NN.nde;
	long double cost = NN.nde->cost + this->dist(x, y, x1, y1);

	for (int i = 0; i < v1.size(); i++) {
		node *nde = v1[i];
		if ((cost > nde->cost + this->dist(x, y, nde->x, nde->y)) && this->obstacleEdge(x, y, nde->x, nde->y)) {
			cost = nde->cost + this->dist(x, y, nde->x, nde->y);
			nde1 = nde;
		}
	}
	x1 = x;
	y1 = y;

	vector<node *> v;
	node *nde = new node(x, y, cost, nde1, v);
	nde1->add_child(nde);
	this->TempPoints->insert(x, y, nde);
	this->tempPointsarr.push_back(nde);
	cv::line(this->treeimage, Point(x, y), Point(nde1->x, nde1->y), Scalar(200));

	for (int i = 0; i < v1.size(); i++) {
		node *nde2 = v1[i];
		if ((nde2->x == nde1->x) && (nde2->y == nde1->y)) {
			continue;
		}
		if (nde2->cost > nde->cost + this->dist(nde2->x, nde2->y, nde->x, nde->y)) {
			if (this->obstacleEdge(nde2->x, nde2->y, nde->x, nde->y)) {
				nde2->cost = nde->cost + this->dist(nde2->x, nde2->y, nde->x, nde->y);
				this->createLinkGoalRooted(nde2, nde);
				nde2->propogateCost();
			}
		}
	}
	nde->propogateCost();
	v1.clear();
	
	showImage("treeimage", this->treeimage);

}

bool workspace::checkTermination(int &x, int &y) {
	vector<node* > v;
	this->Points->searchRNN(x, y, RADIUS, v);
	for (int i = 0; i < v.size(); i++) {
		node* nde1 = v[i];
			if (this->obstacleFreeEdgeGoalBiased(x, y, nde1->x, nde1->y)) {
				v.clear();
				return true;
			}
	}
	v.clear();
	return false;
}

node * workspace::deleteTempSubtree(node *nde) {
	for (int i = 0; i < nde->children.size(); i++) {
		nde->children[i] = this->deleteTempSubtree(nde->children[i]);
	}
	nde->children.clear();
	/*if (!this->TempPoints->ifPresent(nde->x, nde->y))
		cout << "Failure" << endl;
	this->TempPoints = deleteRec(this->TempPoints, nde->x, nde->y);
	if (this->TempPoints->ifPresent(nde->x, nde->y))
		cout << "Failure1" << endl;
	*/
	delete (nde);
	return NULL;
}

node* workspace::validateTempTree(node* nde) {
	for (int i = 0; i < nde->children.size(); i++) {
		node* nde1 = nde->children[i];
		if (obstacleEdge(nde->x, nde->y, nde1->x, nde1->y)) {
			nde->children[i] = this->validateTempTree(nde->children[i]);
		}
		else {
			vector<node* >::iterator it;
			func(nde->children, nde->children[i], it);
			nde->children[i] = this->deleteTempSubtree(nde->children[i]);
			nde->children.erase(it);
		}
	}
	this->tempPointsarr.push_back(nde);
	if (this->TempPoints == NULL) {
		this->TempPoints = new KDTree(nde->x, nde->y, 0, NULL, NULL, nde);
	}
	else {
		if(!this->TempPoints->ifPresent(nde->x, nde->y))
			this->TempPoints->insert(nde->x, nde->y, nde);
	}
	return nde;
}

void workspace::beforePrintTempTree() {
	Mat image = this->treeimage.clone();
	if (this->TempRRT == NULL)
		return;
	queue<node* > q;
	q.push(this->TempRRT);
	node* nde = NULL;
	while (!q.empty()) {
		nde = q.front();
		/*if (!this->TempPoints->ifPresent(nde->x, nde->y)) {
			cout << "not present in print temp tree" << endl;
		}*/
		q.pop();
		for (int i = 0; i < nde->children.size(); i++) {
			node* nde1 = nde->children[i];
			cv::line(image, Point(nde->x, nde->y), Point(nde1->x, nde1->y), Scalar(200));
			q.push(nde->children[i]);
		}
	}
	showImage("image", image);
	//waitKey(0);
	//destroyWindow("image");
}

void workspace::printTempTree() {
	for (int i = 0; i < this->tempPointsarr.size(); i++) {
		node* nde = this->tempPointsarr[i];
		if(nde->parent != NULL)
			cv::line(this->treeimage, Point(nde->x, nde->y), Point(nde->parent->x, nde->parent->y), Scalar(200));
	}
	showImage("treeimage", this->treeimage);
	//waitKey(0);
}

void workspace::findNewRootNode() {
	node* cur = NULL;
	long double cost = LDBL_MAX;
	vector<node*> v;
	for (int i = 0; i < this->tempPointsarr.size(); i++) {
		/*if (this->tempPointsarr[i]->children.size() > 0) {
			continue;
		}*/
		
		node* nde = this->tempPointsarr[i];
		this->Points->searchRNN(nde->x, nde->y, RADIUS, v);
		for (int i = 0; i < v.size(); i++) {
			node* nde1 = v[i];
			if (cost > nde->cost + nde1->cost + this->dist(nde->x, nde->y, nde1->x, nde1->y)) {
				if (this->obstacleFreeEdgeGoalBiased(nde->x, nde->y, nde1->x, nde1->y)) {
					cost = nde->cost + nde1->cost + this->dist(nde->x, nde->y, nde1->x, nde1->y);
					cur = nde1;
				}
			}
		}
		v.clear();
	}
	if (cur == NULL) {
		cout << "Yes NULL " << endl;
		exit(0);
	}
	while (cur->parent->parent != NULL) {
		cv::line(this->treeimage, Point(cur->x, cur->y), Point(cur->parent->x, cur->parent->y), (200, 200, 200));
		showImage("treeimage", this->treeimage);
		cur = cur->parent;
	}
	//waitKey(0);
	this->cx = cur->x;
	this->cy = cur->y;

	this->steps.push_back(make_pair(cur->x, cur->y));
	this->updateRoot(cur);
}

void workspace::deleteTempPoints() {

	node* nde = this->TempRRT;
	if (nde == NULL)
		return;
	queue<node*> q;
	q.push(nde);
	while (!q.empty()) {
		nde = q.front();
		q.pop();
		this->TempPoints = this->TempPoints->deletePoint(nde->x, nde->y);
		for (int i = 0; i < nde->children.size(); i++) {
			q.push(nde->children[i]);
		}
	}
	this->TempPoints = NULL;
}

void workspace::goalRootedRRTStar() {
	int cx = this->dx;
	int cy = this->dy;

	//this->beforePrintTempTree();
	this->deleteTempPoints();
	/*if (this->TempPoints == NULL) {
		cout << "Null ok" << endl;
	}*/

	if (this->TempRRT == NULL) {
		vector<node* > v1;
		node* p1 = NULL;
		this->TempRRT = new node(this->dx, this->dy, 0.0, p1, v1);
		this->TempPoints = new KDTree(this->dx, this->dy, 0, NULL, NULL, this->TempRRT);
	}
	this->tempPointsarr.clear();

	
	this->TempRRT = this->validateTempTree(this->TempRRT);
	//cout << "Validate Tree" << endl;
	this->printTempTree();
	//cout << "tree Printed" << endl;
	
	int i = 0;
	
	while (i < 8) {
		this->addConnectionGoalRooted(cx, cy);
		if (this->checkTermination(cx, cy)) {
		//	cout << "Termination Checked" << endl;
			i++;
		}
	}
	//cout << "Add Connection Complete" << endl;
	this->findNewRootNode();
}

//Goal rooted RRT-star with biasness towards points of original tree
void workspace::type4() {
	//cout << this->points.size() << endl;
	this->goalRootedRRTStar();
}

//Pessimistic Approach
void workspace::type5() {
	this->GoalBiasedBlackRRT();
	node* nde = this->tempGoalNode;
	while (nde->parent != NULL) {
		if (this->Points->ifPresent(nde->x, nde->y)) {
			break;
		}
		nde = nde->parent;
	}
	node* parent = nde->parent;
	node*root = nde;
	nde->parent = NULL;
	int cx1 = nde->x;
	int cy1 = nde->y;
	this->cx = nde->x;
	this->cy = nde->y;
	vector<pair<int, int> > v;
	while (parent != NULL) {
		v.push_back(make_pair(nde->x, nde->y));
		vector<node*>::iterator it;
		func(parent->children, nde, it);
		parent->children.erase(it);
		nde->add_child(parent);
		node* parent1 = parent->parent;
		parent->parent = nde;
		nde = parent;
		parent = parent1;
	}
	//cout << "not updated" << endl;
	this->RRTree = root;
	this->RRTree->cost = 0;
	this->RRTree->propogateCost();
	for (int i = v.size() - 1; i >= 0; i--) {
		
		this->markVisible();
		/*if (this->checkIfGoalVisible()) {
			this->removeGeneratedLeafNodes();
			return;
		}*/
		this->steps.push_back(v[i]);
		this->cx = v[i].first;
		this->cy = v[i].second;
		

	}
	this->cx = cx1;
	this->cy = cy1;
	this->removeGeneratedLeafNodes();
}

void workspace::createLink(node* childlink, node* parentlink) {
	node *oldparent, *child, *parent;
	child = childlink;
	parent = parentlink;
	vector<node*>::iterator it;
	cv::line(this->treeimage, Point(childlink->x, childlink->y), Point(parent->x, parent->y), Scalar(100, 100, 100));

	while (true) {
		oldparent = child->parent;
		func(oldparent->children, child, it);
		oldparent->children.erase(it);
		child->parent = parent;
		parent->children.push_back(child);
		if ((oldparent->cost > child->cost + this->dist(child->x, child->y, oldparent->x, oldparent->y))) {
			oldparent->cost = child->cost + this->dist(child->x, child->y, oldparent->x, oldparent->y);
			parent = child;
			child = oldparent;
		}
		else {
			cv::line(this->treeimage, Point(child->x, child->y), Point(oldparent->x, oldparent->y), Scalar(50, 50, 50));
			break;
		}
	}
}

void workspace::addConnections(int &x1, int &y1) {
	if (this->goalFound && this->goalNode != NULL) {
		node *nde = this->goalNode;
		while (nde->parent != NULL) {
			cv::line(this->treeimage, Point(nde->x, nde->y), Point(nde->parent->x, nde->parent->y), Scalar(100));
			nde = nde->parent;
		}
	}

	int x, y;
	this->generatePoint(x, y);

	NNret NN;
	//int i = 0;
	while (true) {

		int refx = 10000000000000;
		int refy = 10000000000000;
		NN = this->Points->search(x, y, LDBL_MAX, refx, refy, NULL);
		x1 = NN.x;
		y1 = NN.y;
		
		this->stepto(x1, y1, x, y);
		if (!this->Points->ifPresent(x, y)) {
			if (this->obstacleFreeEdge(x, y, x1, y1)) {
				if (!this->check_for_gray(x, y))
					break;
			}
		}
		
		this->generatePoint(x, y);
	}


	vector<node* > v1;
	this->Points->searchRNN(x, y, RADIUS, v1);

	node* nde1 = NN.nde;
	long double cost = NN.nde->cost + this->dist(x, y, x1, y1);

	for (int i = 0; i < v1.size(); i++) {
		node *nde = v1[i];
		if ((cost > nde->cost + this->dist(x, y, nde->x, nde->y)) && this->obstacleFreeEdge(x, y, nde->x, nde->y)) {
			cost = nde->cost + this->dist(x, y, nde->x, nde->y);
			nde1 = nde;
		}
	}
	x1 = x;
	y1 = y;


	vector<node *> v;
	node *nde = new node(x, y, cost, nde1, v);
	nde1->add_child(nde);
	this->Points->insert(x, y, nde);
	cv::line(this->treeimage, Point(x, y), Point(nde1->x, nde1->y), Scalar(100));



	for (int i = 0; i < v1.size(); i++) {
		node *nde2 = v1[i];
		if ((nde2->x == nde1->x) && (nde2->y == nde1->y)) {
			continue;
		}
		if (nde2->cost > nde->cost + this->dist(nde2->x, nde2->y, nde->x, nde->y)) {
			if (this->obstacleFreeEdge(nde2->x, nde2->y, nde->x, nde->y)) {
				nde2->cost = nde->cost + this->dist(nde2->x, nde2->y, nde->x, nde->y);
				//cout << "Yes1" << endl;
				nde2->propogateCost();
				//cout << "Yes" << endl;
				this->createLink(nde2, nde);
			}
		}
	}
	nde->propogateCost();
	v1.clear();
	if (this->goalFound) {
		this->Points->searchRNN(this->dx, this->dy, 4, v1);
		node * nde = v1[0];
		for (int i = 1; i < v1.size(); i++) {
			if (v1[i]->cost < nde->cost) {
				nde = v1[i];
			}
		}
		this->goalNode = nde;
		while (nde->parent != NULL) {
			cv::line(this->treeimage, Point(nde->x, nde->y), Point(nde->parent->x, nde->parent->y), Scalar(200));
			nde = nde->parent;
		}
		v1.clear();
	}

	showImage("treeimage", this->treeimage);
}

void workspace::SimpleRRT() {
	//cout << "Simple RRT-star start" << endl;
	int cx = this->cx;
	int cy = this->cy;
	int i = 0;
	while (i < 150) {
		this->addConnections(cx, cy);
		this->points.push_back(make_pair(cx, cy));
		i++;
		if (!this->goalFound && this->checkifgoalFound(cx, cy)) {
			this->goalFound = true;
		}
	}
}

void workspace::growRRT() {
	this->SimpleRRT();

	if (this->checkifgoalFound(this->cx, this->cy)) {
		return;
	}
	
	if (!this->goalFound) {
		this->type4();
	}
}

double workspace::printFinalPath() {
	double cost = 0;
	for (int i = 1; i < this->steps.size(); i++) {
		pair<int, int> p1 = this->steps[i];
		pair<int, int> p2 = this->steps[i-1];
		cv::line(this->image, Point(p1.first, p1.second), Point(p2.first, p2.second), Scalar(150));
		cost += (sqrt((p1.first - p2.first)*(p1.first - p2.first)*1.0 + (p1.second - p2.second)*(p1.second - p2.second)*1.0));
	}

	cv::line(this->image, Point(this->cx, this->cy), Point(this->dx, this->dy), Scalar(150));
	cost += sqrt((this->cx - this->dx)*(this->cx - this->dx)*1.0 + (this->cy - this->dy)*(this->cy - this->dy)*1.0);

	//cout << "Cost " << cost << endl;
	showImage("Final Path", this->image);
	imshow("Final Path", this->image);
	waitKey(0);
	destroyAllWindows();

	this->steps.clear();

	return cost;

}

node* workspace::deleteWholeSubtree(node* nde) {

	for (int i = 0; i < nde->children.size(); i++) {
		nde->children[i] = this->deleteWholeSubtree(nde->children[i]);
	}
		this->Points = this->Points->deletePoint(nde->x, nde->y);
		nde->children.clear();
		nde->parent = NULL;
		delete (nde);
		return NULL;
}

void workspace::validateTree() {
	node* nde = this->RRTree;
	queue<node*> q;
	q.push(nde);
	while (!q.empty()) {
		nde = q.front();
		q.pop();
		for (int i = 0; i < nde->children.size(); i++) {
			node* nde1 = nde->children[i];
			if (this->obstacleFreeEdge(nde1->x, nde1->y, nde->x, nde->y)) {
			
					q.push(nde1);
			
			}
			else { //Case when point has to be deleted and its children has to be assigned a new parent
			
				vector<node*>::iterator it;
				func(nde->children, nde1, it);
				nde->children.erase(it);
				nde1->parent = NULL;
				nde1 = this->deleteWholeSubtree(nde1);
			}
		}
	}
}

void workspace::printWholeTree() {
	this->treeimage = this->visibleimage.clone();

	node* nde = this->RRTree;
	queue<node*> q;
	q.push(nde);
	while (!q.empty()) {
		nde = q.front();
		q.pop();
		for (int i = 0; i < nde->children.size(); i++) {
			cv::line(this->treeimage, Point(nde->x, nde->y), Point(nde->children[i]->x, nde->children[i]->y), Scalar(100));
			q.push(nde->children[i]);
		}
	}
	for (int i = 1; i < this->steps.size(); i++) {
		pair<int, int> p1 = this->steps[i];
		pair<int, int> p2 = this->steps[i - 1];
		cv::line(this->treeimage, Point(p1.first, p1.second), Point(p2.first, p2.second), Scalar(200));
	}

	showImage("treeimage", this->treeimage);
}

void workspace::findNearestObstacle(int &x, int &y, long double &angle) {
	double x1 = this->cx, y1 = this->cy;
	int rx = this->image.cols;
	int ry = this->image.rows;
	long double theta = PI*angle / 180.0;
	int i = 0;
	Scalar intensity;
	while (x1 < rx - 1 and y1 < ry - 1 and x1 > 0 and y1 > 0) {
		intensity = this->image.at<uchar>(y1, x1);
		if (intensity.val[0] == 255) {
			break;
		}
		else {
			x1 = this->cx + i*sin(theta);
			y1 = this->cy + i*cos(theta);
		}
		i++;
	}
	x = x1;
	y = y1;
}

void workspace::markVisible() {
	Mat visibleimage = Mat(this->image.rows, this->image.cols, CV_8UC1, Scalar(0));

	int px[1084];
	int py[1084];
	int lx = -200, ly = -200;
	int nx, ny;
	for (int i = 0; i < 1084; i++) {

		long double angle = (i*1.0) / 3.0;
		this->findNearestObstacle(nx, ny, angle);
		px[i] = nx;
		py[i] = ny;
		if (i != 0) {
			cv::line(visibleimage, Point(nx, ny), Point(lx, ly), Scalar(50));
		}
		lx = nx;
		ly = ny;
	}
	floodFill(visibleimage, Point(this->cx, this->cy), Scalar(50));
	
	for (int i = 0; i < 1084; i++) {
		circle(visibleimage, Point(px[i], py[i]), 1, Scalar(255), 3);
	}
	bitwise_or(visibleimage, this->visibleimage, this->visibleimage);
	
	showImage("visibleimage", this->visibleimage);

}

bool workspace::checkIfGoalVisible() {
	Scalar intensity = this->visibleimage.at<uchar>(this->dy, this->dx);
	if (intensity.val[0] == 0) {
		return false;
	}
	return true;
}

void workspace::getSnG(int sx, int sy, int dx, int dy) {
	this->sx = sx;
	this->sy = sy;
	this->dx = dx;
	this->dy = dy;
}

void workspace::cleanEveryThing() {
	if (this->TempRRT != NULL) {
		this->TempRRT = this->deleteTempSubtree(this->TempRRT);
		delete (this->TempRRT);
	}
	if (this->RRTree != NULL) {
		this->RRTree = this->deleteTempSubtree(this->RRTree);
		delete(this->RRTree);
	}
	if (this->TempPoints != NULL) {
		this->TempPoints = deleteEverything(this->TempPoints);
	}
	if (this->Points != NULL) {
		this->Points = deleteEverything(this->Points);
	}
	this->points.clear();

}

void workspace::start() {
	this->points.push_back(make_pair(this->sx, this->sy));
	if (!this->checkBoundaries(this->sx, this->sy) || !this->checkBoundaries(this->dx, this->dy)) {
		cout << "Invalid Goal or Destination" << endl;
		exit(0);
	}
	vector<node* > v, v1;
	node* p = NULL;

	this->tempGoalNode = false;
	this->TempRRT = NULL;
	this->RRTree = new node(this->sx, this->sy, 0.0, p, v);
	this->Points = new KDTree(this->sx, this->sy, 0, NULL, NULL, RRTree);
	this->cx = this->sx;
	this->cy = this->sy;
	this->steps.push_back(make_pair(this->cx, this->cy));
	this->visibleimage = Mat(this->image.rows, this->image.cols, CV_8UC1, Scalar(0));

	while (!this->goalFound) {
		//cout << "Marking Started " << endl;
		this->markVisible();
		/*if (this->obstacleFreeEdge(this->cx, this->cy, this->dx, this->dy)) {
			break;
		}*/
		if (this->checkIfGoalVisible()) {
			break;
		}
		//cout << "Marking ended " << endl;
		//this->validateTree();
		this->printWholeTree();
		this->growRRT();
		//cout << this->cx << " " << this->cy << endl;
	}
	this->cleanEveryThing();
	//destroyAllWindows();
}
 