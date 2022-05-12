/*
 #  This is the template for cpp Source Codes of different trajectory
 #  planner in unstructured environments.The randomly generated obstacles are
 #  oriented in different directions.
 */
#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "vec2d.h"
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "Circumscribed_circle.hpp"
#include "AABB.hpp"
#include "OBB.hpp"
#include "Comparative_area.hpp"
#include "HybridAStar.hpp"
#include "D_star_lite.hpp"
//#include "A_star.hpp"
#include <time.h>

using namespace cv;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_;
struct Vehicle_kinematics_ vehicle_kinematics_;
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

vector< vector<math::Vec2d>> obstacles_ = GenerateStaticObstacles_unstructured();

struct A_star_ a_star_;

struct Hybrid_astar_ hybrid_astar_;

struct D_star_lite_ d_star_lite_;

int num_nodes_s = 60;
double margin_obs_ = 0.5;
int Nobs = 40;

// for data from python
// int loaddata();
vector<double> InputData_To_Vector(string filename);

int main(int argc, char** argv)
{
    clock_t start,stop;
    D_star_lite path1;
    path1.replan();
    vector<math::Vec2d> obj(5);
    obj[0].set_x(14);
    obj[0].set_y(14);
    obj[1].set_x(14);
    obj[1].set_y(15);
    obj[2].set_x(15);
    obj[2].set_y(15);
    obj[3].set_x(15);
    obj[3].set_y(14);
    obj[4].set_x(14);
    obj[4].set_y(14);
    obstacles_.push_back(obj);
    path1.add_obstacles(obj);
    start = clock();
    path1.replan();
    //D_star_lite_path path = path1.getpath();
    //time end
    stop = clock();
    D_star_lite_path path = path1.getpath();
    double endtime=(double)(stop-start)/CLOCKS_PER_SEC;
    std::cout << "time: "<<endtime <<std::endl;
    
    vector<double> x = path.x;
    vector<double> y = path.y;
    vector<double> theta = path.theta;
    cout<<x.size()<<endl;
    //cout << "vectorload:" << x[1] << endl;
    //cout << "vectorload:" << y[1] << endl;
    //cout << "vectorload:" << theta[1] << endl;
    printf("%d\n",path.completeness_flag);
    int transmethod_flag = 1;
    double path_length = 40.16013599;
    
    struct Trajectory trajectory = TransformPathToTrajectory(x,y,theta,path_length,transmethod_flag);
    cout << "trajectory test:" << trajectory.x[2] << endl;
    cout << "trajectory test:" << trajectory.y[2] << endl;
    //Point2d
    double varx = 1.2;
    Point test(varx, varx);
    VisualizeStaticResults(trajectory);
    //VisualizeDynamicResults(trajectory);
    cout << "Point test:" << test.x << endl;
    
    
    return 0;
}

