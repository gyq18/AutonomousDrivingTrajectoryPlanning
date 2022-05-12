//
//  HybridAStar.cpp
//  zqc_c++
//
//  Created by 枉叹之 on 2022/4/25.
//

#include "HybridAStar.hpp"
#include <queue>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"
#include <time.h>
#include "AABB.hpp"
#include "Circumscribed_circle.hpp"
#include <opencv2/opencv.hpp>
#include "rs.h"

using namespace cv;

extern vector<vector<math::Vec2d>> obstacles_;

extern struct Vehicle_geometrics_ vehicle_geometrics_;
extern struct Vehicle_kinematics_ vehicle_kinematics_;

priority_queue<Hybrid_AStar_Node, vector<Hybrid_AStar_Node>, cmp> hybrid_astar_openlist;

// Convert actual distance to index value
// 实际距离转换为索引值
math::Vec2d calc_xy_index(math::Vec2d pos)
{
    math::Vec2d idx;
    idx.set_x(ceil((pos.x() - planning_scale_.xmin) / hybrid_astar_.resolution_x));
    idx.set_y(ceil((pos.y() - planning_scale_.ymin) / hybrid_astar_.resolution_y));
    if (idx.x() < 0)
    {
        idx.set_x(0);
    }
    if (idx.x() >= hybrid_astar_.num_nodes_x)
    {
        idx.set_x(hybrid_astar_.num_nodes_x - 1);
    }
    if (idx.y() < 0)
    {
        idx.set_y(0);
    }
    if (idx.y() >= hybrid_astar_.num_nodes_y)
    {
        idx.set_y(hybrid_astar_.num_nodes_y - 1);
    }
    return idx;
}

// Convert actual angle to index value
// 实际角度转换为索引值
double calc_theta_index(double theta)
{
    double thetaid = round(RegulateAngle(theta) / hybrid_astar_.resolution_theta);
    if (thetaid >= hybrid_astar_.num_nodes_theta)
    {
        thetaid = hybrid_astar_.num_nodes_theta - 1;
    }
    return thetaid;
}

// Adjust the angle to 0 to 2 * PI
// 调节角度至 0 到 2 * pi
double RegulateAngle(double angle)
{
    while (angle > 2 * M_PI + 0.000001)
    {
        angle = angle - 2 * M_PI;
    }
    while (angle < -0.000001)
    {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

// Convert index value to actual distance
// 索引值转换为实际距离
math::Vec2d calc_grid_position(math::Vec2d idx)
{
    
    math::Vec2d pos;
    pos.set_x(idx.x() * a_star_.resolution_x);
    pos.set_y(idx.y() * a_star_.resolution_y);
    return pos;
}

// Convert index angle to actual angle
// 索引角度转换为实际角度
double calc_grid_position(double theta_id)
{
    double theta = RegulateAngle(theta_id * hybrid_astar_.resolution_theta);
    return theta;
}

// Calculation of H value
// h值的计算
double calc_heuristic(math::Vec2d p1, math::Vec2d p2)
{
    double h = p1.DistanceTo(p2);
    return h;
}

// Collision detection
// 碰撞检测
bool Is_valied(math::Vec2d pos, double theta)
{
    vector<math::Vec2d> v;
    v = CreateVehiclePolygon(pos.x(), pos.y(), theta);
    math::Vec2d min_point = Min(v);
    math::Vec2d max_point = Max(v);
    if ((min_point.y() <= planning_scale_.ymin) ||
        (min_point.y() >= planning_scale_.ymax) ||
        (max_point.x() <= planning_scale_.xmin) ||
        (max_point.x() >= planning_scale_.xmax))
    {
        return false;
    }
    for (int i = 0; i < obstacles_.size(); i++)
    {
        if (Vehicle_AABB_Collision(pos, 0, obstacles_[i]))
            //        if(Vehicle_Circumscribed_Circle_Collision(pos, 0, obstacles_[i]))
            return false;
    }
    return true;
}
bool Is_valied(double x, double y, double theta)
{
    math::Vec2d pos(x, y);
    return Is_valied(pos, theta);
}

// Simulate displacement per unit time
// 模拟单位时间的位移
Hybrid_AStar_Node SimulateForUnitDistance(math::Vec2d p, double theta, double v, double phi)
{
    int Nfe = 10;
    double hi = hybrid_astar_.simulation_step / Nfe;
    double x = p.x();
    double y = p.y();
    for (int i = 0; i < 10; i++)
    {
        x = cos(theta) * v * hi + x;
        y = sin(theta) * v * hi + y;
        theta = tan(phi) * v / vehicle_geometrics_.vehicle_wheelbase * hi + theta;
    }
    Hybrid_AStar_Node p1;
    p1.p.set_x(x);
    p1.p.set_y(y);
    p1.theta = theta;
    p1.theta_id = calc_theta_index(theta);
    p1.v = v;
    p1.phi = phi;
    p1.idx = calc_xy_index(p1.p);
    return p1;
}

// Equidistant resampling path
// 等距重采样路径
Plan_Path ResamplePathWithEqualDistance(Plan_Path path1)
{
    Plan_Path path2;
    path2.completeness_flag = path1.completeness_flag;
    path2.path_length = path2.completeness_flag;
    for (int i = 1; i < path1.theta.size(); i++)
    {
        while (path1.theta[i] - path1.theta[i - 1] > M_PI)
        {
            path1.theta[i] = path1.theta[i] - 2 * M_PI;
        }
        while (path1.theta[i] - path1.theta[i - 1] < -M_PI)
        {
            path1.theta[i] = path1.theta[i] + 2 * M_PI;
        }
    }
    for (int i = 0; i < path1.theta.size() - 1; i++)
    {
        double distance = sqrt(pow(path1.x[i] - path1.x[i + 1], 2) + pow(path1.y[i] - path1.y[i + 1], 2));
        int LARGE_NUM = round(distance * 100);
        int unitx = (path1.x[i + 1] - path1.x[i]) / LARGE_NUM;
        int unity = (path1.y[i + 1] - path1.y[i]) / LARGE_NUM;
        int unittheta = (path1.theta[i + 1] - path1.theta[i]) / LARGE_NUM;
        
        for (int j = 0; j < LARGE_NUM; j++)
        {
            path2.x.push_back(path1.x[i] + j * unitx);
            path2.y.push_back(path1.y[i] + j * unity);
            path2.theta.push_back(path1.theta[i] + j * unittheta);
        }
        path2.x.push_back(path1.x[i + 1]);
        path2.y.push_back(path1.y[i + 1]);
        path2.theta.push_back(path1.theta[i + 1]);
    }
    return path2;
}

// hybrid A* algorithm
// 混合A* 算法
Plan_Path PlanHybirdAStarPath()
{
    // Function initial settings
    // 函数初始设置
    priority_queue<Hybrid_AStar_Node, vector<Hybrid_AStar_Node>, cmp> tmp;
    hybrid_astar_openlist = tmp;
    vector<math::Vec2d> motion(8);
    motion[0].set_x(1);
    motion[0].set_y(0);
    motion[1].set_x(-1);
    motion[1].set_y(0);
    
    motion[2].set_x(1);
    motion[2].set_y(-vehicle_kinematics_.vehicle_phi_max);
    motion[3].set_x(1);
    motion[3].set_y(vehicle_kinematics_.vehicle_phi_max);
    
    motion[4].set_x(-1);
    motion[4].set_y(-vehicle_kinematics_.vehicle_phi_max);
    motion[5].set_x(-1);
    motion[5].set_y(vehicle_kinematics_.vehicle_phi_max);
    
    for (int i = 0; i < obstacles_.size(); i++)
    {
        for (int j = 0; j < obstacles_[i].size(); j++)
        {
            obstacles_[i][j] = calc_xy_index(obstacles_[i][j]);
            obstacles_[i][j] = calc_grid_position(obstacles_[i][j]);
        }
    }
    bool completeness_flag = false;
    
    vector<vector<vector<Hybrid_AStar_Node>>> hybrid_astar_map_(hybrid_astar_.num_nodes_x, vector<vector<Hybrid_AStar_Node>>(hybrid_astar_.num_nodes_y, vector<Hybrid_AStar_Node>(hybrid_astar_.num_nodes_theta)));
    
    math::Vec2d start_pos(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    Hybrid_AStar_Node start_node;
    math::Vec2d goal_pos(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    Hybrid_AStar_Node goal_node;
    
    start_node.p = start_pos;
    start_node.theta = vehicle_TPBV_.theta0;
    start_node.idx = calc_xy_index(start_pos);
    start_node.theta_id = calc_theta_index(start_node.theta);
    
    goal_node.p = goal_pos;
    goal_node.theta = vehicle_TPBV_.thetatf;
    goal_node.idx = calc_xy_index(goal_pos);
    goal_node.theta_id = calc_theta_index(goal_node.theta);
    
    start_node.g = 0;
    start_node.h = calc_heuristic(start_node.p, goal_node.p);
    start_node.f = start_node.g + a_star_.multiplier_H * start_node.h;
    start_node.parent_idx = start_node.idx;
    start_node.parent_theta_id = start_node.theta_id;
    start_node.open = 1;
    start_node.v = 0;
    start_node.phi = 0;
    
    Hybrid_AStar_Node closest_node = start_node;
    
    hybrid_astar_openlist.push(start_node);
    hybrid_astar_map_[start_node.idx.x()][start_node.idx.y()][start_node.theta_id] = start_node;
    int iter = 0;
    vector<vector<double>> final_rspath;
    bool fianl_flag = true;
    // Start planning
    // 开始规划
    while (!hybrid_astar_openlist.empty() && completeness_flag == false && iter < hybrid_astar_.max_iter)
    {
        iter++;
        Hybrid_AStar_Node cur_node = hybrid_astar_openlist.top();
        
        // If the current node is the target node
        // 如果现节点是目标节点
        if ((iter - 1) % (int)(hybrid_astar_.Nrs) == 0)
        {
            double q0[3] = {cur_node.p.x(), cur_node.p.y(), cur_node.theta};
            double q1[3] = {goal_node.p.x(), goal_node.p.y(), goal_node.theta};
            ReedsSheppStateSpace *r = new ReedsSheppStateSpace;
            final_rspath.clear();
            fianl_flag = true;
            final_rspath = r->xingshensample(q0, q1, hybrid_astar_.simulation_step);
            for (int i = 0; i < final_rspath.size() && fianl_flag; i++)
            {
                fianl_flag = Is_valied(final_rspath[i][0], final_rspath[i][1], final_rspath[i][2]);
            }
            if (fianl_flag == true)
            {
                completeness_flag = true;
                closest_node = cur_node;
                break;
            }
        }
        
        hybrid_astar_openlist.pop();
        hybrid_astar_map_[cur_node.idx.x()][cur_node.idx.y()][cur_node.theta_id].open = 0;
        hybrid_astar_map_[cur_node.idx.x()][cur_node.idx.y()][cur_node.theta_id].close = 1;
        if (cur_node.h + abs(cur_node.theta - goal_node.theta) < closest_node.h + abs(cur_node.theta - goal_node.theta))
        {
            closest_node = cur_node;
        }
        for (int i = 0; i < 8; i++)
        {
            Hybrid_AStar_Node child_node;
            child_node = SimulateForUnitDistance(cur_node.p, cur_node.theta, motion[i].x(), motion[i].y());
            child_node.parent_idx = cur_node.idx;
            child_node.parent_theta_id = cur_node.theta_id;
            child_node.g = cur_node.g + hybrid_astar_.simulation_step + hybrid_astar_.penalty_for_direction_changes * abs(cur_node.v - child_node.v) + hybrid_astar_.penalty_for_steering_changes * abs(cur_node.phi - child_node.phi);
            child_node.h = calc_heuristic(child_node.p, goal_node.p);
            child_node.f = child_node.g + a_star_.multiplier_H * child_node.h;
            // If the child node has been explored ever before, and it has been closed:
            // 如果子节点以前被探测过，并且已经关闭：
            if (hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id].close == 1)
            {
                continue;
            }
            child_node.open = 1;
            // If the previously found parent of the child is not good enough, a change is to be made
            // 如果之前找到的子节点的父节点不够好，就要做出改变
            if (hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id].open == 1)
            {
                if (hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id].f > child_node.f)
                {
                    hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id] = child_node;
                }
                continue;
            }
            // Now the child node is ensured to be newly expanded
            // 现在，子节点被确保是新扩展的
            if (!Is_valied(child_node.p, child_node.theta))
            {
                child_node.close = 1;
                hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id] = child_node;
                continue;
            }
            // Now the child node is both new and collision-free.
            // 现在，子节点是新的且无冲突。
            hybrid_astar_openlist.push(child_node);
            hybrid_astar_map_[child_node.idx.x()][child_node.idx.y()][child_node.theta_id] = child_node;
        }
    }
    
    Plan_Path path;
    Hybrid_AStar_Node final_node = closest_node;
    path.path_length = 0;
    path.completeness_flag = completeness_flag;
    Hybrid_AStar_Node next_node = hybrid_astar_map_[final_node.parent_idx.x()][final_node.parent_idx.y()][final_node.parent_theta_id];
    while (!(next_node.idx == start_node.idx))
    {
        path.x.push_back(final_node.p.x());
        path.y.push_back(final_node.p.y());
        path.theta.push_back(final_node.theta);
        path.path_length = path.path_length + sqrt(pow(final_node.p.x() - next_node.p.x(), 2) + pow(final_node.p.y() - next_node.p.y(), 2));
        final_node = next_node;
        next_node = hybrid_astar_map_[final_node.parent_idx.x()][final_node.parent_idx.y()][final_node.parent_theta_id];
    }
    path.x.push_back(final_node.p.x());
    path.y.push_back(final_node.p.y());
    path.theta.push_back(final_node.theta);
    path.path_length = path.path_length + sqrt(pow(final_node.p.x() - next_node.p.x(), 2) + pow(final_node.p.y() - next_node.p.y(), 2));
    path.x.push_back(next_node.p.x());
    path.y.push_back(next_node.p.y());
    path.theta.push_back(next_node.theta);
    reverse(path.x.begin(), path.x.end());
    reverse(path.y.begin(), path.y.end());
    reverse(path.theta.begin(), path.theta.end());
    for (int i = 0; i < final_rspath.size() && fianl_flag; i++)
    {
        path.x.push_back(final_rspath[i][0]);
        path.y.push_back(final_rspath[i][1]);
        path.theta.push_back(final_rspath[i][2]);
    }
    if (path.completeness_flag)
    {
        path.x.push_back(goal_node.p.x());
        path.y.push_back(goal_node.p.y());
        path.theta.push_back(goal_node.theta);
    }
    path = ResamplePathWithEqualDistance(path);
    return path;
}
