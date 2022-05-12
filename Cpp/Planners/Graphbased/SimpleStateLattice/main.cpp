/*
 * @Author: your name
 * @Date: 2022-04-13 09:20:36
 * @LastEditTime: 2022-04-23 12:51:30
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /cpp code library/main.cpp
 */
/*
#  This is the template for cpp Source Codes of different trajectory
#  planner in unstructured environments.The randomly generated obstacles are
#  oriented in different directions.
*/
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <random>
#include "vec2d.h"

#include <opencv2/opencv.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "state_lattice_planner.h"
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
int Nobs = 0;
// for data from python
int loaddata();
vector<double> InputData_To_Vector(string filename);
vector<vector<math::Vec2d> > obstacles_ ;


int main(int argc, char** argv) {
    // srand(0);
    obstacles_= GenerateStaticObstacles_unstructured();
    // planning_scale_.ymax = 30;
    int transmethod_flag = 1;
    auto start_time = chrono::high_resolution_clock::now();

    PlanningResult result = SimpleStateLatticePlan(1, "../look_up_table_153244.txt", "../actions.txt", "../insert_points.txt");
    auto end_time = chrono::high_resolution_clock::now();
    std::cout << "SimpleStateLatticePlan time consumed: " << 
        chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count() << 
        " ms" <<
        std::endl;
    if (result.is_complete == 0)
    {
        cout<< "Planning failed. Can not reach the destination." << endl;
        return 1;
    }
    
    struct Trajectory trajectory = TransformPathToTrajectory(result.x, result.y, 
        result.theta, result.path_length, transmethod_flag);
    cout << "trajectory test:" << trajectory.x[2] << endl;
    cout << "trajectory test:" << trajectory.y[2] << endl;

    VisualizeDynamicResults(trajectory);

    return 0;
}




int main_bk(int argc, char** argv) {
    //Mat test = imread("test.jpg"); //载入图像到test
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

