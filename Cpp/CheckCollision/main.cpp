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
#include "CheckByAABB.hpp"
#include "CheckByArea.hpp"
#include "CheckByCircle.hpp"
#include "CheckByOBB.hpp"
#include "CheckByMap.hpp"
#include "CheckByLine.hpp"

#include <time.h>

using namespace cv;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_;
struct Vehicle_kinematics_ vehicle_kinematics_;
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

vector<vector<math::Vec2d>> obstacles_ = GenerateStaticObstacles_unstructured();

struct Hybrid_astar_ hybrid_astar_;

vector<vector<int>> costmap_;

int num_nodes_s = 60;
double margin_obs_ = 0.5;
int Nobs = 15;

vector<double> InputData_To_Vector(string filename);

int main(int argc, char **argv)
{
    double x = 25;
    double y = 25;
    double theta = 0;
    double t1, t2;

    t1 = clock();
    bool Is_collision_aabb = CheckByAABB(x, y, theta);
    t2 = clock();
    double time_aabb = (double)(t2 - t1) / CLOCKS_PER_SEC;

    t1 = clock();
    bool Is_collision_obb = CheckByOBB(x, y, theta);
    t2 = clock();
    double time_obb = (double)(t2 - t1) / CLOCKS_PER_SEC;

    t1 = clock();
    bool Is_collision_circle = CheckByCircle(x, y, theta);
    t2 = clock();
    double time_circle = (double)(t2 - t1) / CLOCKS_PER_SEC;

    t1 = clock();
    bool Is_collision_area = CheckByArea(x, y, theta);
    t2 = clock();
    double time_area = (double)(t2 - t1) / CLOCKS_PER_SEC;

    t1 = clock();
    bool Is_collision_line = CheckByLine(x, y, theta);
    t2 = clock();
    double time_line = (double)(t2 - t1) / CLOCKS_PER_SEC;

    t1 = clock();
    bool Is_collision_map = CheckByMap(x, y, theta);
    t2 = clock();
    double time_map = (double)(t2 - t1) / CLOCKS_PER_SEC;

    printf("CheckByAABB   %d\n", Is_collision_aabb);
    printf("time     %2f\n", time_aabb);
    printf("CheckByOBB    %d\n", Is_collision_obb);
    printf("time     %2f\n", time_obb);
    printf("CheckByCircle %d\n", Is_collision_circle);
    printf("time     %2f\n", time_circle);
    printf("CheckByArea   %d\n", Is_collision_area);
    printf("time     %2f\n", time_area);
    printf("CheckByLine   %d\n", Is_collision_line);
    printf("time     %2f\n", time_line);
    printf("CheckByMap    %d\n", Is_collision_map);
    printf("time     %2f\n", time_map);

    struct Trajectory trajectory;
    trajectory.x.push_back(2);
    trajectory.y.push_back(2);
    trajectory.theta.push_back(0);
    trajectory.phi.push_back(0);
    trajectory.a.push_back(0);
    trajectory.omega.push_back(0);
    trajectory.v.push_back(0);

    trajectory.x.push_back(x);
    trajectory.y.push_back(y);
    trajectory.theta.push_back(theta);
    trajectory.phi.push_back(0);
    trajectory.a.push_back(0);
    trajectory.omega.push_back(0);
    trajectory.v.push_back(0);

    trajectory.x.push_back(57);
    trajectory.y.push_back(57);
    trajectory.theta.push_back(0);
    trajectory.phi.push_back(0);
    trajectory.a.push_back(0);
    trajectory.omega.push_back(0);
    trajectory.v.push_back(0);

    VisualizeStaticResults(trajectory);

    return 0;
}
