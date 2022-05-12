/*
#  This is the template for cpp Source Codes of different trajectory
#  planner in unstructured environments.The randomly generated obstacles are
#  oriented in different directions.
*/
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include <Windows.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "node.h"
#include "PlanInformedRRTStarPath.h"
using namespace cv;
/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_;
struct Vehicle_kinematics_ vehicle_kinematics_;
struct Hybrid_astar_ hybrid_astar_;
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

int num_nodes_s = 60;
double margin_obs_ = 0.5;
int Nobs = 40;

vector<math::Vec2d> ResamplePathWithEqualDistance(vector<math::Vec2d> path, int num_nodes_s) {
    vector<math::Vec2d> path_extended;
    vector<math::Vec2d> path_new;
    for (int i = 0; i < path.size() - 1; i++) {
        double distance = sqrt(pow(path[i].x() - path[i + 1].x(), 2) + pow(path[i].y() - path[i + 1].y(), 2));
        int LARGE_NUM = round(distance * 100);
        double unitx = (path[i + 1].x() - path[i].x()) / LARGE_NUM;
        double unity = (path[i + 1].y() - path[i].y()) / LARGE_NUM;
        for (int j = 0; j < LARGE_NUM; j++) {
            math::Vec2d temp(path[i].x() + j * unitx, path[i].y() + j * unity);
            path_extended.push_back(temp);
        }
    }
    path_extended.push_back(path.back());
    for (int k = 0; k < num_nodes_s - 1; k++) {
        int index = round(k * path_extended.size() / (num_nodes_s - 1));
        path_new.push_back(path_extended[index]);
    }
    path_new.push_back(path_extended.back());
    return path_new;
}

// for data from python
int loaddata();
vector<double> InputData_To_Vector(string filename);
vector < vector<math::Vec2d>> obstacles_ = GenerateStaticObstacles_unstructured();
int main(int argc, char** argv) {
    cv::utils::logging::setLogLevel(utils::logging::LOG_LEVEL_SILENT);
    ////Mat test = imread("test.jpg"); //‘ÿ»ÎÕºœÒµΩtest
    ////imshow("test", test);
    ////waitKey(0);
    ////printf("%f\n", vehicle_kinematics_.min_turning_radius);
    ////printf("%f\n", vehicle_TPBV_.phitf);
    //
    //obstacles_ = GenerateStaticObstacles_unstructured();
    //obstacles_ = GenerateStaticObstacles_unstructured();


    //cout << "load data test" << endl;
    //vector<double> x = InputData_To_Vector("./x.txt");
    //vector<double> y = InputData_To_Vector("./y.txt");
    //vector<double> theta = InputData_To_Vector("./theta.txt");
    ////cout << "vectorload:" << x[1] << endl;
    ////cout << "vectorload:" << y[1] << endl;
    ////cout << "vectorload:" << theta[1] << endl;

    //int transmethod_flag = 1;
    //double path_length = 40.16013599;
    // 
    //struct Trajectory trajectory = TransformPathToTrajectory(x,y,theta,path_length,transmethod_flag);
    //cout << "trajectory test:" << trajectory.x[2] << endl;
    //cout << "trajectory test:" << trajectory.y[2] << endl;
    ////Point2d
    //double varx = 1.2;
    //Point test(varx, varx);
    ////VisualizeStaticResults(trajectory);
    //VisualizeDynamicResults(trajectory);
    ////cout << "Point test:" << test.x << endl;
    //return 0;
    obstacles_ = GenerateStaticObstacles_unstructured();
    MyNode start(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    MyNode goal(vehicle_TPBV_.ytf, vehicle_TPBV_.ytf);
    InformedRRTStar rrt(start, goal, obstacles_, planning_scale_);
    DWORD t1, t2;
    t1 = GetTickCount64();
    pair<vector<math::Vec2d>, double> result = rrt.informed_rrt_star_search();
    vector<math::Vec2d> path = result.first;
    double path_length = result.second;
    t2 = GetTickCount64();
    cout << "time = " << t2 - t1 << "ms" << endl;
    cout << path_length << endl;
    path = ResamplePathWithEqualDistance(path, num_nodes_s);
    vector<double> x = {};
    vector<double> y = {};
    vector<double> theta = {};
    cout << path.size() << endl;
    for (int i = 0; i < path.size(); i++) {
        x.push_back(path[path.size() - i -1].x());
        y.push_back(path[path.size() - i - 1].y());
    }
    theta.push_back(vehicle_TPBV_.theta0);
    for (int j = 1; j < path.size(); j++) {
        theta.push_back(atan2(y[j] - y[j - 1], x[j] - x[j - 1]));
    }
    int transmethod_flag = 1;
    struct Trajectory trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag);
    VisualizeDynamicResults(trajectory);
    return 0;
}

