/*
 * @Author: lishengyong
 * @Date: 2022-05-15 16:14:01
 * @LastEditors: lishengyong
 * @LastEditTime: 2022-05-15 16:33:56
 * @FilePath: /cpp code/PlanSpeedForDynamicScenarios.h

 */
#pragma once

#include "statelatticecpp/state_lattice_planner.h"
#include <opencv2/opencv.hpp>
#include "statelatticecpp/Visualize.h"
#include "GenerateDynamicObstacles_unstructured.h"


PlanningResult PlanSpeedForDynamicScenarios(std::vector<double> x, std::vector<double> y, std::vector<double> theta);

std::pair<std::vector<int>, std::vector<int>> SearchVelocityInStGraph(std::vector<double> x, std::vector<double> y, std::vector<double> theta);

class VehiclePolygonFull
{
public:
    std::vector<double> x;
    std::vector<double> y;
};

std::vector<std::pair<int, int>> SearchStPathViaAStar();
int IsVehicleCollidingWithMovingObstacle(double x, double y, double theta, DObstacles obs);
VehiclePolygonFull CreateVehiclePolygonFull(double x, double y, double theta);
std::pair<std::vector<double>, std::vector<double>> OptimizeVelocityInStGraph(std::vector<int> t0, std::vector<int> s0);
PlanningResult TransformPathToTrajectoryForDynamicScenarios(std::vector<double> x, std::vector<double> y, 
    std::vector<double> theta, std::vector<double>t, std::vector<double> s);

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> ResamplePathWithEqualDistance(std::vector<double>x, std::vector<double> y,
 std::vector<double> theta);

 void VisualizeDynamicResultsForDynamicScenarios(PlanningResult& trajectory, std::string filename,
    int frame_speed);
void drawframe_dynamic(PlanningResult trajectory,int i, cv_axis2d axis, cv::Mat canvas);

std::vector<double> solve_qp(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd Aineq, Eigen::MatrixXd bineq, Eigen::MatrixXd Aeq,
    Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s0);