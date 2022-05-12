#include <ctime>
#include <algorithm>
#include "PlanInformedRRTStarPath.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <random>
#include "vec2d.h"
#include <sys/timeb.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "node.h"
using namespace std;

double* InformedRRTStar::sample_unit_ball() {
	struct timeb timeSeed;
	ftime(&timeSeed);
	srand(timeSeed.time * 1000 + timeSeed.millitm);
	double a = rand() % 100;
	double b = rand() % 100;
	if (b < a) {
		double t = a;
		a = b;
		b = t;
	}
	static double sample[2] = { b * cos(2 * M_PI * a / b), b * sin(2 * M_PI * a / b) };
	return sample;
}

double* InformedRRTStar::sample_free_space() {
	double x_min = this->planning_scale_.xmin;
	double x_max = this->planning_scale_.xmax;
	double y_min = this->planning_scale_.ymin;
	double y_max = this->planning_scale_.ymax;
	static double rnd[2];
	struct timeb timeSeed;
	ftime(&timeSeed);
	srand(timeSeed.time * 1000 + timeSeed.millitm);
	int a = rand() % 100;
	if (a > this->goal_sample_rate) {
		rnd[0] = x_min + rand() % 100 / 100.0 * (x_max - x_min);
		rnd[1] = y_min + rand() % 100 / 100.0 * (y_max - y_min);
	}
	else {
		rnd[0] = this->goal.x;
		rnd[1] = this->goal.y;
	}
	return rnd;
}

double* InformedRRTStar::informed_sample(double cMax, double cMin, double* xCenter, double e_theta) {
	static double *rnd;
	if (cMax < double(RAND_MAX)) {
		double* xBall = this->sample_unit_ball();
		*(rnd) = cos(e_theta) * cMax / 2.0 * *(xBall) - sin(e_theta) * sqrt(pow(cMax, 2) - pow(cMin, 2)) / 2.0 * *(xBall + 1) + *(xCenter);
		*(rnd + 1) = sin(e_theta) * cMax / 2.0 * *(xBall) - cos(e_theta) * sqrt(pow(cMax, 2) - pow(cMin, 2)) / 2.0 * *(xBall + 1) + *(xCenter + 1);
	}
	else {
		rnd = this->sample_free_space();
	}
	return rnd;
}

int InformedRRTStar::get_nearest_list_index(double* rnd) {
	vector<MyNode> nodes = this->node_list;
	double min_dis = double(RAND_MAX);
	int min_index = 0;
	for (int i = 0; i < nodes.size(); i++) {
		double temp_dis = pow(nodes[i].x - *(rnd), 2) + pow(nodes[i].y - *(rnd + 1), 2);
		if (temp_dis < min_dis) {
			min_dis = temp_dis;
			min_index = i;
		}
	}
	return min_index;
}

MyNode InformedRRTStar::get_new_node(double theta, int n_ind, MyNode nearestNode) {
	MyNode newNode(nearestNode.x + this->expandDis * cos(theta), nearestNode.y + this->expandDis * sin(theta));
	newNode.cost = nearestNode.cost + this->expandDis;
	newNode.parent = n_ind;
	return newNode;
}

bool InformedRRTStar::check_move(double x_1, double y_1, double x_2, double y_2) {
	math::Vec2d from_loc(x_1, y_1);
	math::Vec2d to_loc(x_2, y_2);
	vector < vector<math::Vec2d>> obstacles_ = this->obstacleList;
	auto iter = obstacles_.begin();
	for (; iter < obstacles_.end(); iter++)
	{
		// we assume that the from_pos is valid.
		if (checkObj_point(to_loc, *iter))
		{
			return false;
		}

		// TODO: it seems I can check all obstacles in one time.
		if (checkObj_linev(from_loc, to_loc, *iter))
		{
			return false;
		}
	}
	return true;
}

bool InformedRRTStar::check_move2(double x_1, double y_1, double x_2, double y_2) {
math::Vec2d from_loc(x_1, y_1);
math::Vec2d to_loc(x_2, y_2);
vector < vector<math::Vec2d>> obstacles_ = this->obstacleList;
auto iter = obstacles_.begin();
for (; iter < obstacles_.end(); iter++)
{
	// TODO: it seems I can check all obstacles in one time.
	if (checkObj_linev(from_loc, to_loc, *iter))
	{
		return false;
	}
}
return true;
}

vector<int> InformedRRTStar::find_near_nodes(MyNode newNode) {
	vector<int> near_inds = {};
	vector<MyNode> nodes = this->node_list;
	int n_node = nodes.size();
	double r = 50 * sqrt(log(n_node) / n_node);
	for (int i = 0; i < n_node; i++) {
		if (pow(nodes[i].x - newNode.x, 2) + pow(nodes[i].y - newNode.y, 2) < pow(r, 2)) {
			near_inds.push_back(i);
		}
	}
	return near_inds;
}

MyNode InformedRRTStar::choose_parent(MyNode newNode, vector<int> near_inds) {
	if (near_inds.size() == 0) {
		return newNode;
	}
	else {
		vector<double> dList = {};
		auto iter = near_inds.begin();
		for (; iter < near_inds.end(); iter++) {
			MyNode near_node = this->node_list[*iter];
			if (this->check_move2(newNode.x, newNode.y, near_node.x, near_node.y)) {
				double dis = sqrt(pow(near_node.x - newNode.x, 2) + pow(near_node.y - newNode.y, 2));
				dList.push_back(near_node.cost + dis);
			}
			else {
				dList.push_back(double(RAND_MAX));
			}
		}
		auto minPosition = min_element(dList.begin(), dList.end());
		int min_ind = near_inds[minPosition - dList.begin()];
		if (*minPosition == double(RAND_MAX)) {
			return newNode;
		}
		else {
			newNode.cost = *minPosition;
			newNode.parent = min_ind;
			return newNode;
		}
	}
}

vector<math::Vec2d> InformedRRTStar::get_final_course(int lastIndex) {
	vector<math::Vec2d> path = {};
	math::Vec2d goal(this->goal.x, this->goal.y);
	path.push_back(goal);
	while (this->node_list[lastIndex].parent != -1) {
		MyNode node = this->node_list[lastIndex];
		math::Vec2d temp(node.x, node.y);
		path.push_back(temp);
		lastIndex = node.parent;
	}
	math::Vec2d start(this->start.x, this->start.y);
	path.push_back(start);
	return path;
}

pair<vector<math::Vec2d>, double> InformedRRTStar::informed_rrt_star_search() {
	MyNode start = this->start;
	MyNode goal = this->goal;
	this->node_list.push_back(this->start);
	double cBest = double(RAND_MAX);
	vector<math::Vec2d> path = {};
	double cMin = sqrt(pow(start.x - goal.x, 2) + pow(start.y - goal.y, 2));

	double xCenter[2];
	xCenter[0] = (start.x + goal.x) / 2.0;
	xCenter[1] = (start.y + goal.y) / 2.0;
	double e_theta = atan2(goal.y - start.y, goal.x - start.x);

	for (int i = 0; i < this->maxIter; i++) {
		double* rnd = this->informed_sample(cBest, cMin, xCenter, e_theta);
		int n_ind = this->get_nearest_list_index(rnd);
		MyNode nearestNode = this->node_list[n_ind];
		double theta = atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x);

		MyNode newNode = this->get_new_node(theta, n_ind, nearestNode);
		bool noCollision = this->check_move(nearestNode.x, nearestNode.y, newNode.x, newNode.y);

		if (noCollision) {
			vector<int> nearInds = this->find_near_nodes(newNode);
			newNode = this->choose_parent(newNode, nearInds);
			this->node_list.push_back(newNode);

			double d_goal = sqrt(pow((newNode.x - goal.x), 2) + pow((newNode.y - goal.y), 2));
			if (d_goal < this->expandDis) {
				if (this->check_move2(newNode.x, newNode.y, goal.x, goal.y)) {
					int lastIndex = this->node_list.size() - 1;
					double tempPathLen = newNode.cost + d_goal;

					if (tempPathLen < cBest) {
						path = this->get_final_course(lastIndex);
						cBest = tempPathLen;
					}
				}
			}
		}
	}
	return make_pair(path, cBest);
}

