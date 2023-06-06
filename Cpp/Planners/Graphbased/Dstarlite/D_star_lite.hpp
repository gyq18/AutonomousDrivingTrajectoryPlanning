//  D_star_lite.hpp


#ifndef D_star_lite_hpp
#define D_star_lite_hpp

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <queue>
#include <cmath>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "vec2d.h"
#include "mathstruct.h"
#include "AABB.hpp"
#include "Circumscribed_circle.hpp"

using namespace cv;

extern struct D_star_lite_ d_star_lite_;
extern struct Vehicle_TPBV_ vehicle_TPBV_;
extern struct Planning_scale_ planning_scale_;
extern int num_nodes_s;
extern double margin_obs_;
extern int Nobs;

struct State
{
    math::Vec2d idx;
    math::Vec2d parent_idx;
    double state;
    string t;
    double h;
    double k;
    State()
    {
        this->state = 0;
        this->t ="new";
        this->h = 0;
        this->k = 0;
    }
    friend bool operator ==(const State &a,const State &b)
    {
        if(a.idx == b.idx)
            return true;
        else
            return false;
    }


    // Movement consumption between two adjacent points
    // 相邻两点间的移动消耗
    double cost(State state1)
    {
    	if ((this->state > 0 && this->state <= 2) || (state1.state > 0 && state1.state <= 2))
        {
            return DBL_MAX;
        }
        if(this->idx.x()==state1.idx.x()||this->idx.y()==state1.idx.y())
            return 1;
        return 1.414;
    }
    void set_state(double state1)
    {
        this->state = state1;
    }
};

struct D_star_lite_path
{
    vector<double> x;
    vector<double> y;
    vector<double> theta;
    double path_length;
    bool completeness_flag;
    
};

class D_star_lite
{
public:
    // Add obstacles
    // 增加障碍物
    void add_obstacles(vector<math::Vec2d> obj);
    // Re planning
    //重新规划
    bool replan();
    // Output path
    // 输出路径
    D_star_lite_path getpath();
    
private:
    vector<vector<State>> d_star_lite_map;
    list<State> d_star_lite_openlist;
    vector<math::Vec2d> motion;
    D_star_lite_path path;
    D_star_lite_path id_path;
    // Initialize
    // 初始化
    void initialization();
    
    // Design 01 map
    // 设计 01 地图
    void costmap();
    
    // Convert actual distance to index value
    // 实际距离转换为索引值
    math::Vec2d calc_xy_index(math::Vec2d pos);
    // Convert index value to actual distance
    // 索引值转换为实际距离
    math::Vec2d calc_grid_position(math::Vec2d idx);
    
    // Get 8 neighborhood
    // 获取8邻域
    vector<State> get_neighbers(math::Vec2d idx);
    
    void remove(State state1);
    void insert(State state1,double h_new);

    State get_min_state();
	// It uses an openlist to recurse the cost on the whole path through the parent
	// 是以一个openlist，通过parent递推整条路径上的cost
    void modify_cost(State state1);
    // When the obstacle changes, push back from the target point to the starting point
    // to update the change of path cost caused by the change of obstacle
    // 当障碍物发生变化时，从目标点往起始点回推，更新由于障碍物发生变化而引起的路径代价的变化
    void modify(State state1);
    // Main process of D * algorithm
    // D*算法的主要过程
    double process_state();
};

#endif /* D_star_lite_hpp */
