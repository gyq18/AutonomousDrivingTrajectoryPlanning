#pragma once
#include <vector>
#include "mathstruct.h"
using namespace std;
extern struct Vehicle_geometrics_ vehicle_geometrics_;
extern struct Vehicle_kinematics_ vehicle_kinematics_;
extern struct Planning_scale_ planning_scale_;
extern double resolution_x;
extern double resolution_y;
extern int num_nodes_x;
extern int num_nodes_y;
struct Trajectory {
	vector<double> x, y, theta, v, a, phi, omega;
};


struct Trajectory TransformPathToTrajectory(vector<double> x, vector<double> y, vector<double> theta, double path_length, int method_flag);
// method_flag:1-FulfillProfiles(x, y, theta);2-GYQProfiles

vector<vector<double>> FulfillProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length);
vector<vector<double>> GYQProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length);
math::Vec2d calc_xy_index(math::Vec2d pos);
math::Vec2d calc_xy_index(double x,double y);
math::Vec2d calc_grid_position(math::Vec2d idx);