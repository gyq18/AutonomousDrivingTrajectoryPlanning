//
//  HybridAStar.hpp
//  zqc_c++
//
//  Created by 枉叹之 on 2022/4/25.
//

#ifndef HybridAStar_hpp
#define HybridAStar_hpp

#include <stdio.h>
#include <iostream>
#include <queue>
#include <cmath>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"
#include "AABB.hpp"
#include <opencv2/opencv.hpp>
#include "Circumscribed_circle.hpp"
#include "rs.h"

using namespace cv;

extern struct Vehicle_TPBV_ vehicle_TPBV_;
extern struct Planning_scale_ planning_scale_;
extern int num_nodes_s;
extern double margin_obs_;
extern int Nobs;

extern struct Hybrid_astar_ hybrid_astar_;

struct Hybrid_AStar_Node
{
    math::Vec2d p; // 实际位置 Actual location
    double theta;  // theta
    double f;
    double g;
    double h;
    bool open;       // 在开放列表中 In the open list
    bool close;      // 在封闭列表中 In closed list
    math::Vec2d idx; // 该节点的矩阵索引 Own matrix index
    int theta_id;
    math::Vec2d parent_idx; // 父节点的矩阵索引 Matrix index of parent node
    int parent_theta_id;
    // expansion operation [v,phi] that yields the current node
    // 产生当前节点的扩展操作[v，phi]
    double v;
    double phi;
    Hybrid_AStar_Node()
    {
        this->open = 0;
        this->close = 0;
    }
};

// Sorting function
// 排序函数
struct cmp
{
    bool operator()(Hybrid_AStar_Node &a, Hybrid_AStar_Node &b)
    {
        return a.f > b.f;
    }
};

struct Plan_Path
{
    vector<double> x;
    vector<double> y;
    vector<double> theta;
    double path_length;
    bool completeness_flag;
};

extern vector<vector<Hybrid_AStar_Node>> hybrid_astar_map_;
extern priority_queue<Hybrid_AStar_Node, vector<Hybrid_AStar_Node>, cmp> hybrid_astar_openlist;

// Convert actual distance to index value
// 实际距离转换为索引值
math::Vec2d calc_xy_index(math::Vec2d pos);

// Convert actual angle to index value
// 实际角度转换为索引值
double calc_theta_index(double theta);

// Adjust the angle to 0 to 2 * PI
// 调节角度至 0 到 2 * pi
double RegulateAngle(double angle);

// Convert index value to actual distance
// 索引值转换为实际距离
math::Vec2d calc_grid_position(math::Vec2d idx);

// Convert index angle to actual angle
// 索引角度转换为实际角度
double calc_grid_position(double theta_id);

// Calculation of H value
// h值的计算
double calc_heuristic(math::Vec2d p1, math::Vec2d p2);

// Collision detection
// 碰撞检测
bool Is_valied(math::Vec2d pos, double theta);
bool Is_valied(double x, double y, double theta);

// Simulate displacement per unit time
// 模拟单位时间的位移
Hybrid_AStar_Node SimulateForUnitDistance(math::Vec2d p, double theta, double v, double phi);

// Equidistant resampling path
// 等距重采样路径
Plan_Path ResamplePathWithEqualDistance(Plan_Path path1);

// The starting angle needs to be between 0 and 2 * PI
// hybrid A* algorithm
// 起始角度需在 0 到 2 * pi 之间
// 混合A* 算法
Plan_Path PlanHybirdAStarPath();

#endif /* HybridAStar_hpp */
