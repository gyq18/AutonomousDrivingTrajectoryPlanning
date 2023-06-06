/*
#  This is the template for cpp Source Codes of different trajectory
#  planner in unstructured environments.The randomly generated obstacles are
#  oriented in different directions.
*/
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include <opencv2/opencv.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "PlanPRMPath.h"
#include "bezier.h"
#include "fstream"
using namespace cv;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_ = Vehicle_geometrics_();
struct Vehicle_kinematics_ vehicle_kinematics_ = Vehicle_kinematics_();
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

int num_nodes_s = 60;
double resolution_x = 0.1;
double resolution_y = 0.1;
int num_nodes_x = ceil(planning_scale_.x_scale/resolution_x)+1;
int num_nodes_y = ceil(planning_scale_.y_scale/resolution_y)+1;
double margin_obs_ = 0.5;
int Nobs = 20;

vector < vector<math::Vec2d>> obstacles_;
vector<math::Vec2d> path;
cv::Mat costmap_;

int main(int argc, char** argv) {
    
    obstacles_ = GenerateStaticObstacles_unstructured();
    costmap_ = CreateCostmap();
    clock_t t1,t2,t3,t4;
    t1 = clock();
    pair<struct Trajectory, double> ans = PlanPRMPath();
    struct Trajectory trajectory = ans.first;
    
    t2 = clock();
    //VisualizeStaticResults(trajectory);
    std::cout<<"PRM time: " <<t2-t1<<std::endl;
    double path_length = ans.second;
    for (int i = 0; i < trajectory.x.size(); i++) {
        math::Vec2d node(trajectory.x[i], trajectory.y[i]);
        path.push_back(node);
    }
    t3 = clock();
    Bezier::bezier B_curves(path);
    vector<Bezier::bounding_box> box_list = B_curves.get_box_list();
    double obj = 0;
    int res = B_curves.BezierPloyCoeffGeneration(obj);
    struct Trajectory bezier_path = B_curves.get_bezier_path(path_length);
    t4 = clock();
    // ofstream open("../out/total.csv", ios::out | ios::app);
    // if (open.is_open()){
    //     open << t2-t1 << "," << t4-t3 << endl;
    //     open.close();
    // }
    std::cout<<"bezier time: " <<t4-t3<<std::endl;
    VisualizeStaticResults(bezier_path, box_list);
    VisualizeDynamicResults(bezier_path);
    return 0;
}