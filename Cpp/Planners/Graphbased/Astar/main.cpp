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
int Nobs = 2;

// for data from python
int loaddata();
vector<double> InputData_To_Vector(string filename);
vector < vector<math::Vec2d>> obstacles_ = GenerateStaticObstacles_unstructured();
int main(int argc, char** argv) {
    //Mat test = imread("test.jpg"); //‘ÿ»ÎÕºœÒµΩtest
    //imshow("test", test);
    //waitKey(0);
    //printf("%f\n", vehicle_kinematics_.min_turning_radius);
    //printf("%f\n", vehicle_TPBV_.phitf);
    
    obstacles_ = GenerateStaticObstacles_unstructured();
    obstacles_ = GenerateStaticObstacles_unstructured();


    cout << "load data test" << endl;
    vector<double> x = InputData_To_Vector("./x.txt");
    vector<double> y = InputData_To_Vector("./y.txt");
    vector<double> theta = InputData_To_Vector("./theta.txt");
    //cout << "vectorload:" << x[1] << endl;
    //cout << "vectorload:" << y[1] << endl;
    //cout << "vectorload:" << theta[1] << endl;

    int transmethod_flag = 1;
    double path_length = 40.16013599;
     
    struct Trajectory trajectory = TransformPathToTrajectory(x,y,theta,path_length,transmethod_flag);
    cout << "trajectory test:" << trajectory.x[2] << endl;
    cout << "trajectory test:" << trajectory.y[2] << endl;
    //Point2d
    double varx = 1.2;
    Point test(varx, varx);
    VisualizeStaticResults(trajectory);
    VisualizeDynamicResults(trajectory);
    cout << "Point test:" << test.x << endl;
    return 0;
}

