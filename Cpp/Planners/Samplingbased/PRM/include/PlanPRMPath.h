#pragma once
#include "TransformP2T.h"
#include "vec2d.h"
#include <opencv2/opencv.hpp>

class AstarNode
{
public:
	int vertex_order;
	double g;
	double h;
	double f;
	AstarNode* parent;
public:
	AstarNode(int vertex_order_ = 0, double g_ = 0, double h_ = 0, double f_ = 0) :vertex_order(vertex_order_), g(g_), h(h_), f(f_), parent(nullptr) {

	}
	bool operator<(const AstarNode& a) {
		return f > a.f;
	}
};

extern vector<vector<math::Vec2d>> obstacles_;
extern double resolution_x;
extern double resolution_y;
extern int num_nodes_x;
extern int num_nodes_y;
extern cv::Mat costmap_;
extern struct Vehicle_geometrics_ vehicle_geometrics_;
extern struct Planning_scale_ planning_scale_;
cv::Mat CreateCostmap();

vector<math::Vec2d> getvertex(unsigned int k);
vector<vector<int>> getedges(vector<math::Vec2d> vertex);
pair <struct Trajectory, double> PlanPRMPath();
vector<math::Vec2d> Resample(vector<math::Vec2d> vec);
bool find(priority_queue<AstarNode*> openlist, int vertex_order);
vector<double> linspace(double pos1, double pos2, int n);
bool Is2DNodeValid(double x, double y);
bool Is3DNodeValid(double x, double y, double theta);
bool IsCross(vector<math::Vec2d> vehicle, vector<math::Vec2d> obstacle);