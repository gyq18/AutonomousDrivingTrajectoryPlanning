#pragma once
#include <vector>
#include "mathstruct.h"
using namespace std;
extern struct Vehicle_geometrics_ vehicle_geometrics_;
extern struct Vehicle_kinematics_ vehicle_kinematics_;

struct Trajectory {
	vector<double> x, y, theta, v, a, phi, omega;
};

struct Trajectory TransformPathToTrajectory(vector<double> x, vector<double> y, vector<double> theta, double path_length, int method_flag);
// method_flag:1-FulfillProfiles(x, y, theta);2-GYQProfiles

vector<vector<double>> FulfillProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length);
vector<vector<double>> GYQProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length);



