/*
 * @Author: lishengyong 
 * @Date: 2022-05-15 14:13:24
 * @LastEditors: lishengyong 
 * @LastEditTime: 2022-05-15 16:26:26
 * @FilePath: /cpp code/main.cpp

 */

/*
//  This is the template for cpp Source Codes of different trajectory
//  planner in unstructured environments.The randomly generated obstacles are
//  oriented in different directions.
*/
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <random>
#include "vec2d.h"
#
#include <opencv2/opencv.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "GenerateDynamicObstacles_unstructured.h"
#include "global_structs.h"
#include "statelatticecpp/state_lattice_planner.h"
#include "PlanSpeedForDynamicScenarios.h"

using namespace cv;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_;
struct Vehicle_kinematics_ vehicle_kinematics_;
struct Hybrid_astar_ hybrid_astar_;
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

class StGraphSearch st_graph_search_;

int num_nodes_s = 60;

double margin_obs_ = 0.5;
int Nobs = 0;
// for data from python
int loaddata();
vector<double> InputData_To_Vector(string filename);
vector<vector<math::Vec2d> > obstacles_ ;
Eigen::MatrixXd costmap_;

vector<vector<DObstacles>> dynamic_obs;




int main()
{
    // gloabl configing
    extern Vehicle_geometrics_ vehicle_geometrics_;
    extern Vehicle_kinematics_ vehicle_kinematics_;
    extern Planning_scale_ planning_scale_;
    extern Vehicle_TPBV_ vehicle_TPBV_;

    obstacles_ = GenerateStaticObstacles_unstructured();
    vehicle_geometrics_.vehicle_wheelbase = 2.8;  // L_W,wheelbase of the ego vehicle (m)
    vehicle_geometrics_.vehicle_front_hang = 0.96; // L_F,front hang length of the ego vehicle (m)
    vehicle_geometrics_.vehicle_rear_hang = 0.929; // L_R,rear hang length of the ego vehicle (m)
    vehicle_geometrics_.vehicle_width = 1.942; // width of the ego vehicle (m)
    vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang; // length of the ego vehicle (m)
    vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
    vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
    vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
    // vehicle kinematics settings
    vehicle_kinematics_.vehicle_v_max = 2.5; 
    vehicle_kinematics_.vehicle_v_min = -2.5; // upper and lower bounds of v(t) (m/s)
    vehicle_kinematics_.vehicle_a_max = 0.5; 
    vehicle_kinematics_.vehicle_a_min = -0.5; // upper and lower bounds of a(t) (m/s^2)
    vehicle_kinematics_.vehicle_jerk_max = 0.5; 
    vehicle_kinematics_.vehicle_jerk_min = -0.5; // upper and lower bounds of jerk(t) (m/s^3) 
    vehicle_kinematics_.vehicle_phi_max = 0.7;
    vehicle_kinematics_.vehicle_phi_min = -0.7; // upper and lower bounds of phi(t) (rad)
    vehicle_kinematics_.vehicle_omega_max = 0.5; 
    vehicle_kinematics_.vehicle_omega_min = -0.5; // upper and lower bounds of omega(t) (rad/s)
    vehicle_kinematics_.min_turning_radius = vehicle_geometrics_.vehicle_wheelbase/tan(vehicle_kinematics_.vehicle_phi_max);

    // scenario settings
    planning_scale_.xmin=0;
    planning_scale_.xmax=60;
    planning_scale_.ymin=0;
    planning_scale_.ymax=60; //space is a rectange, [lx,ux],[ly,uy]
    // planning_scale_.x_scale = planning_scale_.xmax - planning_scale_.xmin;
    // planning_scale_.y_scale = planning_scale_.ymax - planning_scale_.ymin;

    // vehicle initial and terminal states settings
    vehicle_TPBV_.x0=4;
    vehicle_TPBV_.y0=52;
    vehicle_TPBV_.theta0=0;
    vehicle_TPBV_.v0=0;
    vehicle_TPBV_.phi0=0;
    vehicle_TPBV_.a=0;
    vehicle_TPBV_.omega0=0;
    vehicle_TPBV_.xtf=52;
    vehicle_TPBV_.ytf=4;
    vehicle_TPBV_.thetatf=M_PI/2;
    vehicle_TPBV_.vtf=0;
    vehicle_TPBV_.phitf=0;
    vehicle_TPBV_.a=0;
    vehicle_TPBV_.omegatf=0;

    PlanningResult res = SimpleStateLatticePlan(1, "statelatticecpp/look_up_table_153244.txt", 
        "statelatticecpp/insert_points.txt");

    double path_length = res.path_length;
    st_graph_search_.k = 2;
    st_graph_search_.num_nodes_t = 120;
    st_graph_search_.num_nodes_s = num_nodes_s;
    st_graph_search_.penalty_for_inf_velocity = 4;
    st_graph_search_.max_t = round(path_length * 2);
    st_graph_search_.max_s = path_length;
    st_graph_search_.resolution_s = path_length / st_graph_search_.num_nodes_s;
    st_graph_search_.resolution_t = st_graph_search_.max_t / st_graph_search_.num_nodes_t;

    dynamic_obs = GenerateDynamicObstacles_unstructured();
    if (!res.is_complete)
    {
        printf("Planning failed\n");
        exit(1);
    }
    
    auto plan_res = PlanSpeedForDynamicScenarios(res.x, res.y, res.theta);
    
    VisualizeDynamicResultsForDynamicScenarios(plan_res, "video2.mp4", 24);
    

    return 0;
}