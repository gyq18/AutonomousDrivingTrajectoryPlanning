//  A_star.hpp


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
    math::Vec2d p; // Actual location
    double theta;  // theta
    double f;
    double g;
    double h;
    bool open;              // In the open list
    bool close;             //  In closed list
    math::Vec2d idx;        //  Own matrix index
    math::Vec2d parent_idx; //  Matrix index of parent node
    double parent_theta;
    A_Star_Node()
    {
        this->open = 0;
        this->close = 0;
    }
};

// Sorting function
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
math::Vec2d calc_xy_index(math::Vec2d pos);

// Calculation of H value
double calc_heuristic(math::Vec2d p1, math::Vec2d p2);

// Convert index value to actual distance
math::Vec2d calc_grid_position(math::Vec2d idx);

// Find extreme endpoint
vector<math::Vec2d> extremum_node(math::Vec2d idx);

// Design 01 map
vector<vector<vector<int>>> costmap(vector<vector<vector<int>>> obj_map_);

// A* algorithm
A_star_path PlanAStarPath();

#endif /* A_star_hpp */
