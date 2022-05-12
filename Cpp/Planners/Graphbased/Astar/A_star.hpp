//
//  A_star.hpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/23.
//

#ifndef A_star_hpp
#define A_star_hpp

#include <stdio.h>
#include <iostream>
#include <queue>
#include <cmath>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

extern struct Hybrid_astar_ hybrid_astar_;
extern struct Vehicle_TPBV_ vehicle_TPBV_;
extern struct Planning_scale_ planning_scale_;
extern int num_nodes_s;
extern double margin_obs_;
extern int Nobs;

struct A_Star_Node
{
    math::Vec2d p; // 实际位置 Actual location
    double theta;  // theta
    double f;
    double g;
    double h;
    bool open;              // 在开放列表中 In the open list
    bool close;             // 在封闭列表中 In closed list
    math::Vec2d idx;        // 该节点的矩阵索引 Own matrix index
    math::Vec2d parent_idx; // 父节点的矩阵索引 Matrix index of parent node
    double parent_theta;
    A_Star_Node()
    {
        this->open = 0;
        this->close = 0;
    }
};

// Sorting function
// 排序函数
struct cmp
{
    bool operator()(A_Star_Node &a, A_Star_Node &b)
    {
        return a.f > b.f;
    }
};

struct A_star_path
{
    vector<double> x;
    vector<double> y;
    vector<double> theta;
    double path_length;
    bool completeness_flag;
};

extern vector<vector<A_Star_Node>> astar_map_;
extern vector<vector<vector<int>>> obj_map_;
extern priority_queue<A_Star_Node, vector<A_Star_Node>, cmp> astar_openlist;
extern priority_queue<A_Star_Node, vector<A_Star_Node>, cmp> astar_closelist;

// Convert actual distance to index value
// 实际距离转换为索引值
math::Vec2d calc_xy_index(math::Vec2d pos);

// Calculation of H value
// h值的计算
double calc_heuristic(math::Vec2d p1, math::Vec2d p2);

// Convert index value to actual distance
// 索引值转换为实际距离
math::Vec2d calc_grid_position(math::Vec2d idx);

// Find extreme endpoint
// 找到极值端点
vector<math::Vec2d> extremum_node(math::Vec2d idx);

// Design 01 map
// 设计 01 地图
vector<vector<vector<int>>> costmap(vector<vector<vector<int>>> obj_map_);

// A* algorithm
// A* 算法
A_star_path PlanAStarPath();

#endif /* A_star_hpp */
