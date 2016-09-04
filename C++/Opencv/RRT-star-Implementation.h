#ifndef RRTSTAR
#define RRTSTAR

#include<bits\stdc++.h>
#include<opencv2\highgui\highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "KdTree.h"
#include "RRT.h"
#define EPSILON 30
#define RADIUS 35
#define PI 3.14159265358979323846
using namespace std;
using namespace cv;

class workspace {
public:
	Mat image, visibleimage, treeimage, tempimage;
	int sx, sy, dx, dy, cx, cy;
	bool flag;
	node *RRTree , *TempRRT;
	KDTree *Points, *TempPoints;
	bool goalFound, tempGoalFound;
	node* goalNode, *tempGoalNode;
	vector<node* > tempPointsarr;
	vector<node* > archievedTree;
	vector<pair<int, int> > steps;
	vector<pair<int, int> > points;

	workspace();
	void getSourceandGoal();
	void start();
	bool checkifgoalFound(int &x, int &y);
	bool checkBoundaries(int &x, int &y);
	void addConnections(int &x, int &y);
	void generatePoint(int &x, int &y);
	bool obstacleFree(int &x, int &y);
	void stepto(int &x1, int &y1, int &x2, int  &y2);
	bool obstacleFreeEdge(int &x1, int &y1, int &x2, int &y2);
	bool check_same(int &x1, int &y1, int &x2, int &y2);
	long double dist(int &x1, int &y1, int &x2, int &y2);
	void createLink(node* childlink, node* parentlink);
	void growRRT();
	void SimpleRRT();
	void GoalBiasedBlackRRT();
	void markVisible();
	void findNearestObstacle(int &x, int &y, long double &angle);
	void printWholeTree();
	bool check_for_black(int &x1, int &y1, int &x2, int &y2);
	bool check_for_gray(int &x, int &y);
	void generateGoalBiasedPoint(int &x, int &y);
	bool checkForTempPoints(int &x, int &y);
	void updateRoot(node* root);
	void removeGeneratedLeafNodes();
	double printFinalPath();
	bool obstacleFreeGoalBiased(int &x, int &y);
	bool obstacleFreeEdgeGoalBiased(int &x1, int &y1, int &x2, int &y2);
	void validateTree();
	void goalRootedRRTStar();
	node* deleteWholeSubtree(node* nde);
	void removeConnectionOfGeneratedNodes();
	void useArchieveTree();
	void printBlackSubtree(node* nde);
	void deleteBlackSubtree(node* nde);
	void takeNextStep();
	void addConnectionGoalRooted(int &x, int &y);
	void createLinkGoalRooted(node *childlink, node * parentlink);
	bool nonObstacleFree(int &x, int &y);
	bool obstacleEdge(int &x, int &y, int &x1, int &y1);
	void generateGoalRootedPoint(int &x, int &y);
	bool checkTermination(int &x, int &y);
	node* validateTempTree(node* nde);
	node* deleteTempSubtree(node *nde);
	void findNewRootNode();
	bool checkIfGoalVisible();
	bool checkWithinRadius(int &x, int &y);
	void printTempTree();
	void beforePrintTempTree();
	void deleteTempPoints();
	void getSnG(int sx, int sy, int dx, int dy);
	void cleanEveryThing();
	void type1();
	void type2();
	void type3();
	void type4();
	void type5();
};
#endif // !RRTSTAR