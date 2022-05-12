//
//  D_star_lite.cpp
//  zqc_c++
//
//  Created by 枉叹之 on 2022/4/29.
//

#include "D_star_lite.hpp"
#include <queue>
#include <iostream>
#include <math.h>
#include <time.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "vec2d.h"
#include "mathstruct.h"
#include "AABB.hpp"
#include "Circumscribed_circle.hpp"
#include <string.h>

using namespace cv;

extern vector<vector<math::Vec2d>> obstacles_;

// Convert actual distance to index value
// 实际距离转换为索引值
math::Vec2d D_star_lite::calc_xy_index(math::Vec2d pos)
{
    math::Vec2d idx;
    idx.set_x(ceil((pos.x() - planning_scale_.xmin) / d_star_lite_.resolution_x));
    idx.set_y(ceil((pos.y() - planning_scale_.ymin) / d_star_lite_.resolution_y));
    return idx;
}

// Convert index value to actual distance
// 索引值转换为实际距离
math::Vec2d D_star_lite::calc_grid_position(math::Vec2d idx)
{
    
    math::Vec2d pos;
    pos.set_x(idx.x() * d_star_lite_.resolution_x);
    pos.set_y(idx.y() * d_star_lite_.resolution_y);
    return pos;
}

// Get 8 neighborhood
// 获取8邻域
vector<State> D_star_lite::get_neighbers(math::Vec2d idx)
{
    vector<State> state_list;
    for (int i = 0; i < 8; i++)
    {
        math::Vec2d cur_idx;
        cur_idx = idx + motion[i];
        if (cur_idx.x() < 0 || cur_idx.y() < 0 || cur_idx.x() >= d_star_lite_.num_nodes_x || cur_idx.y() >= d_star_lite_.num_nodes_y)
        {
            continue;
        }
        state_list.push_back(d_star_lite_map[cur_idx.x()][cur_idx.y()]);
    }
    return state_list;
}

void D_star_lite::remove(State state1)
{
    d_star_lite_openlist.remove(d_star_lite_map[state1.idx.x()][state1.idx.y()]);
    if (d_star_lite_map[state1.idx.x()][state1.idx.y()].t == "open")
    {
        d_star_lite_map[state1.idx.x()][state1.idx.y()].t = "close";
    }
}

void D_star_lite::insert(State state1, double h_new)
{
    if (d_star_lite_map[state1.idx.x()][state1.idx.y()].t == "new")
        d_star_lite_map[state1.idx.x()][state1.idx.y()].k = h_new;
    else if (d_star_lite_map[state1.idx.x()][state1.idx.y()].t == "open")
        d_star_lite_map[state1.idx.x()][state1.idx.y()].k = min(d_star_lite_map[state1.idx.x()][state1.idx.y()].k, h_new);
    else if (d_star_lite_map[state1.idx.x()][state1.idx.y()].t == "close")
        d_star_lite_map[state1.idx.x()][state1.idx.y()].k = min(d_star_lite_map[state1.idx.x()][state1.idx.y()].h, h_new);
    d_star_lite_map[state1.idx.x()][state1.idx.y()].h = h_new;
    d_star_lite_map[state1.idx.x()][state1.idx.y()].t = "open";
    d_star_lite_openlist.push_back(d_star_lite_map[state1.idx.x()][state1.idx.y()]);
}

State D_star_lite::get_min_state()
{
    State min_s;
    min_s.state = -1;
    if (d_star_lite_openlist.empty())
        return min_s;
    double k_min = DBL_MAX;
    for (auto iter1 = d_star_lite_openlist.begin(); iter1 != d_star_lite_openlist.end(); iter1++)
    {
        if (k_min > iter1->k)
        {
            k_min = iter1->k;
            min_s = *iter1;
        }
    }
    return min_s;
}

// It uses an openlist to recurse the cost on the whole path through the parent
// 是以一个openlist，通过parent递推整条路径上的cost
void D_star_lite::modify_cost(State state1)
{
    if (d_star_lite_map[state1.idx.x()][state1.idx.y()].t == "close")
    {
        math::Vec2d parent_id = d_star_lite_map[state1.idx.x()][state1.idx.y()].parent_idx;
        insert(d_star_lite_map[state1.idx.x()][state1.idx.y()], d_star_lite_map[parent_id.x()][parent_id.y()].h + d_star_lite_map[state1.idx.x()][state1.idx.y()].cost(d_star_lite_map[parent_id.x()][parent_id.y()]));
    }
}

// When the obstacle changes, push back from the target point to the starting point
// to update the change of path cost caused by the change of obstacle
// 当障碍物发生变化时，从目标点往起始点回推，更新由于障碍物发生变化而引起的路径代价的变化
void D_star_lite::modify(State state1)
{
    modify_cost(d_star_lite_map[state1.idx.x()][state1.idx.y()]);
    double time_start = (double)(clock());
    while (1)
    {
        double time_end = (double)(clock());
        if ((time_end - time_start) / CLOCKS_PER_SEC > 10)
            return;
        double k_min = process_state();
        if ((k_min == -1) || (k_min >= d_star_lite_map[state1.idx.x()][state1.idx.y()].h))
        {
            break;
        }
    }
}


// Add obstacles
// 增加障碍物
void D_star_lite::add_obstacles(vector<math::Vec2d> obj)
{
    double x_r = ceil((vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang) / d_star_lite_.resolution_x) + 1;
    double x_l = ceil(vehicle_geometrics_.vehicle_rear_hang / d_star_lite_.resolution_x) + 1;
    double dy = ceil(vehicle_geometrics_.vehicle_width / 2 / d_star_lite_.resolution_y) + 1;
    double minx;
    double miny;
    double maxx;
    double maxy;
    math::Vec2d id;
    vector<math::Vec2d> obj_id;
    id = calc_xy_index(obj[0]);
    obj_id.push_back(id);
    minx = id.x();
    miny = id.y();
    maxx = id.x();
    maxy = id.y();
    for (int j = 1; j < 4; j++)
    {
        id = calc_xy_index(obj[j]);
        obj_id.push_back(id);
        if (minx > id.x())
            minx = id.x();
        if (maxx < id.x())
            maxx = id.x();
        if (miny > id.y())
            miny = id.y();
        if (maxy < id.y())
            maxy = id.y();
    }
    obj_id.push_back(id);
    for (int m = minx; m < maxx + 1; m++)
    {
        for (int n = miny; n < maxy + 1; n++)
        {
            if (PtInPolygon(m, n, obj_id))
            {
                d_star_lite_map[m][n].set_state(1);
                for (int i = m - x_r - 1; i < m + x_l + 1; i++)
                {
                    if (i >= 0 && i < d_star_lite_.num_nodes_x)
                    {
                        for (int j = n - dy - 1; j < n + dy + 1; j++)
                        {
                            if (j >= 0 && j < d_star_lite_.num_nodes_y && d_star_lite_map[i][j].state == 0)
                            {
                                d_star_lite_map[i][j].set_state(2);
                            }
                        }
                    }
                }
            }
        }
    }
    Nobs++;
}

// Design 01 map
// 设计 01 地图
void D_star_lite::costmap()
{
    double minx;
    double miny;
    double maxx;
    double maxy;
    double x_r = ceil((vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang) / d_star_lite_.resolution_x) + 1;
    double x_l = ceil(vehicle_geometrics_.vehicle_rear_hang / d_star_lite_.resolution_x) + 1;
    double dy = ceil(vehicle_geometrics_.vehicle_width / 2 / d_star_lite_.resolution_y) + 1;
    math::Vec2d id;
    for (int i = 0; i < obstacles_.size(); i++)
    {
        vector<math::Vec2d> obj;
        id = calc_xy_index(obstacles_[i][0]);
        minx = id.x();
        miny = id.y();
        maxx = id.x();
        maxy = id.y();
        obj.push_back(id);
        for (int j = 1; j < 4; j++)
        {
            id = calc_xy_index(obstacles_[i][j]);
            obj.push_back(id);
            if (minx > id.x())
                minx = id.x();
            if (maxx < id.x())
                maxx = id.x();
            if (miny > id.y())
                miny = id.y();
            if (maxy < id.y())
                maxy = id.y();
        }
        // Set the position of the obstacle to 1
        // 将障碍物的位置设成1
        for (int m = minx; m < maxx + 1; m++)
        {
            for (int n = miny; n < maxy + 1; n++)
            {
                if (PtInPolygon(m, n, obj))
                    d_star_lite_map[m][n].set_state(1);
            }
        }
    }
    // Set border to 1
    // 将边框设成1
    for (int i = 0; i < d_star_lite_.num_nodes_x; i++)
    {
        d_star_lite_map[i][0].set_state(1);
        d_star_lite_map[i][d_star_lite_.num_nodes_y - 1].set_state(1);
    }
    for (int j = 0; j < d_star_lite_.num_nodes_y; j++)
    {
        d_star_lite_map[0][j].set_state(1);
        d_star_lite_map[d_star_lite_.num_nodes_x - 1][j].set_state(1);
    }
    // Set the point where the vehicle will collide to 2
    // 将车辆会有碰撞的点设为2
    for (int i = 0; i < d_star_lite_.num_nodes_x; i++)
    {
        for (int j = 0; j < d_star_lite_.num_nodes_y; j++)
        {
            d_star_lite_map[i][j].idx.set_x(i);
            d_star_lite_map[i][j].idx.set_y(j);
            if (d_star_lite_map[i][j].state == 1)
            {
                for (int m = i - x_r - 1; m < i + x_l + 1; m++)
                {
                    if (m >= 0 && m < d_star_lite_.num_nodes_x)
                        for (int n = j - dy - 1; n < j + dy + 1; n++)
                        {
                            if (n >= 0 && n < d_star_lite_.num_nodes_y && d_star_lite_map[m][n].state == 0)
                            {
                                d_star_lite_map[m][n].set_state(2);
                            }
                        }
                }
            }
        }
    }
}

// Initialize
// 初始化
void D_star_lite::initialization()
{
    // Function initial settings
    // 函数初始设置
    vector<math::Vec2d> motion_(8);
    motion = motion_;
    motion[0].set_x(1);
    motion[0].set_y(0);
    motion[1].set_x(0);
    motion[1].set_y(1);
    motion[2].set_x(-1);
    motion[2].set_y(0);
    motion[3].set_x(0);
    motion[3].set_y(-1);
    
    motion[4].set_x(-1);
    motion[4].set_y(-1);
    motion[5].set_x(-1);
    motion[5].set_y(1);
    motion[6].set_x(1);
    motion[6].set_y(-1);
    motion[7].set_x(1);
    motion[7].set_y(1);
    
    vector<vector<State>> temp_map_(d_star_lite_.num_nodes_x, vector<State>(d_star_lite_.num_nodes_y));
    d_star_lite_map = temp_map_;
    costmap();
}

// Main process of D * algorithm
// D*算法的主要过程
double D_star_lite::process_state()
{
    // Get the minimum k node in the open list
    // 获取open list列表中最小k的节点
    State x = get_min_state();
    if (d_star_lite_map[x.idx.x()][x.idx.y()].state == -1)
        return -1;
    double k_old = d_star_lite_map[x.idx.x()][x.idx.y()].k;
    // Remove from openlist
    // 从openlist中移除
    remove(d_star_lite_map[x.idx.x()][x.idx.y()]);
    vector<State> neig = get_neighbers(x.idx);
    if (k_old < d_star_lite_map[x.idx.x()][x.idx.y()].h)
    {
        for (int y = 0; y < neig.size(); y++)
        {
            if (neig[y].h <= k_old && d_star_lite_map[x.idx.x()][x.idx.y()].h > neig[y].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(neig[y]))
            {
                d_star_lite_map[x.idx.x()][x.idx.y()].parent_idx = neig[y].idx;
                d_star_lite_map[x.idx.x()][x.idx.y()].h = neig[y].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(neig[y]);
            }
        }
    }
    else if (k_old == d_star_lite_map[x.idx.x()][x.idx.y()].h)
    {
        for (int y = 0; y < neig.size(); y++)
        {
            if (d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].t == "new" || (d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx == x.idx && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h != d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(neig[y])) || (!(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx == x.idx) && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h > d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()])))
            {
                d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx = x.idx;
                insert(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()], d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()]));
            }
        }
    }
    else
    {
        for (int y = 0; y < neig.size(); y++)
        {
            if (d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].t == "new" || (d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx == d_star_lite_map[x.idx.x()][x.idx.y()].idx && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h != d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()])))
            {
                d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx = x.idx;
                insert(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()], d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()]));
            }
            else
            {
                if (!(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx == x.idx) && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h > d_star_lite_map[x.idx.x()][x.idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()]))
                {
                    insert(d_star_lite_map[x.idx.x()][x.idx.y()], d_star_lite_map[x.idx.x()][x.idx.y()].h);
                }
                else if (!(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].parent_idx == x.idx) && d_star_lite_map[x.idx.x()][x.idx.y()].h > d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h + d_star_lite_map[x.idx.x()][x.idx.y()].cost(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()]) && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].t == "close" && d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h > k_old)
                {
                    insert(d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()], d_star_lite_map[neig[y].idx.x()][neig[y].idx.y()].h);
                }
                
            }
        }
    }
    return get_min_state().k;
}

// Re planning
//重新规划
bool D_star_lite::replan()
{
    if (d_star_lite_map.size() == 0)
        initialization();
    double time_start = (double)(clock());
    id_path.completeness_flag = false;
    math::Vec2d start_pos(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    math::Vec2d goal_pos(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    math::Vec2d start_idx = calc_xy_index(start_pos);
    math::Vec2d end_idx = calc_xy_index(goal_pos);
    State start = d_star_lite_map[start_idx.x()][start_idx.y()];
    State end = d_star_lite_map[end_idx.x()][end_idx.y()];
    vector<double>().swap(id_path.x);
    vector<double>().swap(id_path.y);
    vector<double>().swap(id_path.theta);
    d_star_lite_openlist.push_back(end);
    while (1)
    {
        double flag = process_state();
        if(flag==-1)
            return false;
        if (d_star_lite_map[start.idx.x()][start.idx.y()].t == "close")
            break;
    }
    id_path.x.push_back(start_idx.x());
    id_path.y.push_back(start_idx.y());
    id_path.theta.push_back(0);
    if (d_star_lite_map[start.idx.x()][start.idx.y()].state > 0)
    {
        id_path.completeness_flag = false;
        return false;
    }
    State cur = d_star_lite_map[start.idx.x()][start.idx.y()];
    /*
     Start from the starting point and travel to the target point. When encountering obstacles,
     modify the cost again and find the path again
     从起始点开始，往目标点行进，当遇到障碍物时，重新修改代价，再寻找路径
     */
    while (!(cur.idx == end.idx))
    {
        double time_end = (double)(clock());
        if ((time_end - time_start) / CLOCKS_PER_SEC > 10)
        {
            id_path.completeness_flag = false;
            return false;
        }
        if (d_star_lite_map[cur.idx.x()][cur.idx.y()].state > 0)
        {
            id_path.completeness_flag = false;
            return false;
        }
        id_path.x.push_back(cur.idx.x());
        id_path.y.push_back(cur.idx.y());
        id_path.theta.push_back(0);
        math::Vec2d parent_id = d_star_lite_map[cur.idx.x()][cur.idx.y()].parent_idx;
        if (d_star_lite_map[parent_id.x()][parent_id.y()].state > 0 && d_star_lite_map[parent_id.x()][parent_id.y()].state <= 2)
        {
            modify(d_star_lite_map[cur.idx.x()][cur.idx.y()]);
            continue;
        }
        
        cur = d_star_lite_map[parent_id.x()][parent_id.y()];
    }
    if (cur.idx == end.idx)
    {
        id_path.completeness_flag = true;
        id_path.x.push_back(end.idx.x());
        id_path.y.push_back(end.idx.y());
        id_path.theta.push_back(0);
    }
    return id_path.completeness_flag;
}

// Output path
// 输出路径
D_star_lite_path D_star_lite::getpath()
{
    vector<double>().swap(path.x);
    vector<double>().swap(path.y);
    vector<double>().swap(path.theta);
    path.completeness_flag = id_path.completeness_flag;
    path.path_length = 0;
    math::Vec2d id(id_path.x[0], id_path.y[0]);
    math::Vec2d pos = calc_grid_position(id);
    path.x.push_back(pos.x());
    path.y.push_back(pos.y());
    path.theta.push_back(0);
    // Convert matrix index to actual map1
    // 矩阵索引转为实际地图
    for (int i = 1; i < id_path.x.size(); i++)
    {
        math::Vec2d id(id_path.x[i], id_path.y[i]);
        math::Vec2d pos = calc_grid_position(id);
        path.x.push_back(pos.x());
        path.y.push_back(pos.y());
        path.theta.push_back(0);
        path.path_length = path.path_length + sqrt(pow((path.x.at(path.x.size() - 1) - path.x.at(path.x.size() - 2)), 2) + pow((path.y.at(path.y.size() - 1) - path.y.at(path.y.size() - 2)), 2));
    }
    return path;
}
