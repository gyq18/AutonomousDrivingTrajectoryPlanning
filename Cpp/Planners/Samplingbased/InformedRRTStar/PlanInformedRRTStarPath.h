#pragma once
#include <vector>
#include <ctime>
#include "mathstruct.h"
#include "node.h"
using namespace std;

struct InformedRRTStar {
/*     
    start: the starting position of trajectory
	goal : the end point of trajectory
	obstacleList : globalvar.obstacles
	planning_scale_ : globalvar.planning_scale_
	expandDis : the distance of expansion in every step, adjustment for planning_scale_
	goal_sample_rate : the probability of directly expanding to the end
	maxIter : the maximum times of iterations, adjustment for the complexity of the graph
	node_list : the list of nodes in the path to the end
*/
	MyNode start, goal;
	vector < vector<math::Vec2d>> obstacleList;
	struct Planning_scale_ planning_scale_;
	double expandDis;
	int goal_sample_rate, maxIter;
	vector<MyNode> node_list;

	InformedRRTStar(MyNode Start, MyNode Goal, vector < vector<math::Vec2d>> obs, struct Planning_scale_ planning_scale_) {
		this->start = Start;
		this->goal = Goal;
		this->obstacleList = obs;
		this->planning_scale_ = planning_scale_;
		this->expandDis = 5;
		this->goal_sample_rate = 10;
		this->maxIter = 200;
		this->node_list = {};
	}
	
	double* sample_unit_ball();

	// generate a random point when no path has been searched
	double* sample_free_space();

	// generate new points in an ellipse
	double* informed_sample(double cMax, double cMin, double* xCenter, double e_theta);

	// for the new point, find the nearest Node in node_list
	int get_nearest_list_index(double* rnd);

	// generate a new Node based on the new point
	MyNode get_new_node(double theta, int n_ind, MyNode nearestNode);

	// A simple function to judge whether there are obstacles between (x1, y1) to (x2, y2).
	bool check_move(double x_1, double y_1, double x_2, double y_2);

	// another check_move method when (x2, y2) is known legal, each parameters are the same as check_move
	bool check_move2(double x_1, double y_1, double x_2, double y_2);

	// find near nodes of newNode
	vector<int> find_near_nodes(MyNode newNode);

	// choose parent for the newNode
	MyNode choose_parent(MyNode newNode, vector<int> near_inds);
	
	// get a path through the parent of Nodes
	vector<math::Vec2d> get_final_course(int lastIndex);

	// use informed_rrt_star method to search for path
	pair<vector<math::Vec2d>, double> informed_rrt_star_search();
};
