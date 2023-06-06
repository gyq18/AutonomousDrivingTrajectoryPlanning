#include "vec2d.h"
#include "TransformP2T.h"
#include <vector>
using namespace std;


struct Trajectory TransformPathToTrajectory(vector<double> x, vector<double> y, vector<double> theta, double path_length, int method_flag) {
	struct Trajectory trajectory;
	trajectory.x = x;
	trajectory.y = y;
	if (method_flag == 1) {
		vector<vector<double>> ans = FulfillProfiles(x, y, theta, path_length);
		trajectory.theta = ans[0];
		trajectory.v = ans[1];
		trajectory.a = ans[2];
		trajectory.phi = ans[3];
		trajectory.omega = ans[4];

	}
	else if (method_flag == 2) {
		vector<vector<double>> ans = GYQProfiles(x, y, theta, path_length);
		trajectory.theta = ans[0];
		trajectory.v = ans[1];
		trajectory.a = ans[2];
		trajectory.phi = ans[3];
		trajectory.omega = ans[4];
	}
	return trajectory;
}
vector<vector<double>> FulfillProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length) {
	vector<vector<double>> ans;
	int Nfe = x.size();
	vector<double> vdr(Nfe);
	for (int ii = 1; ii < Nfe - 1; ++ii) {
		if ((x[ii + 1] - x[ii]) * cos(theta[ii]) + (y[ii + 1] - y[ii]) * sin(theta[ii])) {
			vdr[ii] = 1;
		}
		else {
			vdr[ii] = -1;
		}
	}
	vector<double> v(Nfe);
	vector<double> a(Nfe);
	double dt = path_length / Nfe;
	for (int ii = 1; ii < Nfe; ++ii) {
		v[ii] = vdr[ii] * sqrt(((x[ii] - x[ii - 1]) / dt) * ((x[ii] - x[ii - 1]) / dt) + ((y[ii] - y[ii - 1]) / dt) * ((y[ii] - y[ii - 1]) / dt));
	}
	for (int ii = 1; ii < Nfe; ++ii) {
		a[ii] = (v[ii] - v[ii - 1]) / dt;
	}
	vector<double> phi(Nfe);
	vector<double> omega(Nfe);

	double phi_max = vehicle_kinematics_.vehicle_phi_max;
	double omega_max = vehicle_kinematics_.vehicle_omega_max;
	for (int ii = 1; ii < Nfe - 1; ++ii) {
		phi[ii] = atan((theta[ii + 1] - theta[ii]) * vehicle_geometrics_.vehicle_wheelbase / (dt * v[ii]));
		if (phi[ii] > phi_max) {
			phi[ii] = phi_max;
		}
		else {
			if (phi[ii] < -phi_max) {
				phi[ii] = -phi_max;
			}
		}
	}
	for (int ii = 1; ii < Nfe - 1; ++ii) {
		omega[ii] = (phi[ii + 1] - phi[ii]) / dt;
		if (omega[ii] > omega_max) {
			omega[ii] = omega_max;
		}
		else {
			if (omega[ii] < -omega_max) {
				omega[ii] = -omega_max;
			}
		}
	}
	ans.push_back(theta);
	ans.push_back(v);
	ans.push_back(a);
	ans.push_back(phi);
	ans.push_back(omega);
	return ans;
}
vector<vector<double>> GYQProfiles(vector<double> x, vector<double> y, vector<double> theta, double path_length) {
	vector<vector<double>> ans;
	int Nfe = x.size();
	return ans;
}

math::Vec2d calc_xy_index(math::Vec2d pos)
{
    math::Vec2d idx;
    idx.set_x(ceil((pos.x() - planning_scale_.xmin) / resolution_x));
    idx.set_y(ceil((pos.y() - planning_scale_.ymin) / resolution_y));
    if (idx.x() < 0)
    {
        idx.set_x(0);
    }
    if (idx.x() >= num_nodes_x)
    {
        idx.set_x(num_nodes_x - 1);
    }
    if (idx.y() < 0)
    {
        idx.set_y(0);
    }
    if (idx.y() >= num_nodes_y)
    {
        idx.set_y(num_nodes_y - 1);
    }
    return idx;
}

math::Vec2d calc_xy_index(double x,double y){
	math::Vec2d pos(x,y);
	return calc_xy_index(pos);
}
math::Vec2d calc_grid_position(math::Vec2d idx)
{
    
    math::Vec2d pos;
    pos.set_x(idx.x() * resolution_x);
    pos.set_y(idx.y() * resolution_y);
    return pos;
}