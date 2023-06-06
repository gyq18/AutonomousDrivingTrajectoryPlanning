//  A_star.cpp

#include "A_star.hpp"
#include <queue>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"
#include <time.h>

extern vector<vector<math::Vec2d>> obstacles_;

priority_queue<A_Star_Node, vector<A_Star_Node>, cmp> astar_openlist;

// Convert actual distance to index value
math::Vec2d calc_xy_index(math::Vec2d pos)
{
    math::Vec2d idx;
    idx.set_x(ceil(pos.x() / a_star_.resolution_x));
    idx.set_y(ceil(pos.y() / a_star_.resolution_y));
    return idx;
}

// Calculation of H value
double calc_heuristic(math::Vec2d p1, math::Vec2d p2)
{
    double h = abs(p2.x() - p1.x()) + abs(p2.y() - p1.y());
    return h;
}

// Convert index value to actual distance
math::Vec2d calc_grid_position(math::Vec2d idx)
{
    math::Vec2d pos;
    pos.set_x(idx.x() * a_star_.resolution_x);
    pos.set_y(idx.y() * a_star_.resolution_y);
    return pos;
}

// Find extreme endpoint
vector<math::Vec2d> extremum_node(math::Vec2d idx)
{
    math::Vec2d pos;
    vector<math::Vec2d> p1p2(2);
    pos = calc_grid_position(idx);
    if ((pos.y() - vehicle_geometrics_.vehicle_width / 2 <= planning_scale_.ymin) ||
        (pos.y() + vehicle_geometrics_.vehicle_width / 2 >= planning_scale_.ymax) ||
        (pos.x() - vehicle_geometrics_.vehicle_rear_hang <= planning_scale_.xmin) ||
        (pos.x() + vehicle_geometrics_.vehicle_front_hang +
         vehicle_geometrics_.vehicle_wheelbase >=
         planning_scale_.xmax))
    {
        p1p2[0].set_x(-1);
        return p1p2;
    }
    // collision check
    vector<math::Vec2d> V = CreateVehiclePolygon(pos.x(), pos.y(), 0);
    for (int i = 0; i < 4; i++)
    {
        V[i] = calc_xy_index(V[i]);
    }
    int minx = V[0].x();
    int miny = V[0].y();
    int maxx = V[0].x();
    int maxy = V[0].y();
    for (int i = 1; i < 4; i++)
    {
        if (minx > V[i].x())
            minx = V[i].x();
        if (maxx < V[i].x())
            maxx = V[i].x();
        if (miny > V[i].y())
            miny = V[i].y();
        if (maxy < V[i].y())
            maxy = V[i].y();
    }
    p1p2[0].set_x(minx);
    p1p2[0].set_y(miny);
    p1p2[1].set_x(maxx);
    p1p2[1].set_y(maxy);
    return p1p2;
}

// Design 01 map
vector<vector<vector<int>>> costmap(vector<vector<vector<int>>> obj_map_)
{
    double minx;
    double miny;
    double maxx;
    double maxy;
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
        for (int m = minx; m < maxx; m++)
        {
            for (int n = miny; n < maxy; n++)
            {
                if (PtInPolygon(m, n, obj))
                    obj_map_[m][n][0] = 1;
            }
        }
    }
    for (int i = 1; i < a_star_.num_nodes_x; i++)
    {
        for (int j = 1; j < a_star_.num_nodes_y; j++)
        {
            obj_map_[i][j][1] = obj_map_[i - 1][j][1] + obj_map_[i][j - 1][1] + obj_map_[i][j][0] - obj_map_[i - 1][j - 1][1];
        }
    }
    return obj_map_;
}

A_star_path PlanAStarPath()
{
    priority_queue<A_Star_Node, vector<A_Star_Node>, cmp> tmp;
    astar_openlist = tmp;
    // Function initial settings
    vector<math::Vec2d> motion(8);
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
    
    double motion_length[8] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
    //    {{1,0,1},{0,1,1},{-1,0,1},{0,-1,1},
    //    {-1,-1,(sqrt(2))},{-1,1,sqrt(2)},{1,-1,sqrt(2)},{1,1,sqrt(2)}};
    bool completeness_flag = false;
    
    vector<vector<A_Star_Node>> astar_map_(a_star_.num_nodes_x, vector<A_Star_Node>(a_star_.num_nodes_y));
    vector<vector<vector<int>>> obj_map_(a_star_.num_nodes_x, vector<vector<int>>(a_star_.num_nodes_y, vector<int>(2, 0)));
    obj_map_ = costmap(obj_map_);
    
    math::Vec2d start_pos(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    A_Star_Node start_node;
    math::Vec2d goal_pos(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    A_Star_Node goal_node;
    
    start_node.p = start_pos;
    start_node.theta = 0;
    start_node.idx = calc_xy_index(start_pos);
    start_node.g = 0;
    start_node.h = calc_heuristic(start_node.idx, goal_node.idx);
    start_node.f = start_node.g + a_star_.multiplier_H_for_A_star * start_node.h;
    start_node.parent_idx = start_node.idx;
    
    A_Star_Node closest_node = start_node;
    
    goal_node.p = goal_pos;
    goal_node.idx = calc_xy_index(goal_pos);
    goal_node.theta = 0;
    
    // extern vector < vector<math::Vec2d>> obstacles_;
    // extern vector<vector<vector<int>>> astar_map_;
    // extern priority_queue<Node, vector<Node>, cmp> astar_openlist;
    // extern priority_queue<Node, vector<Node>, cmp> astar_closelist;
    start_node.open = 1;
    astar_openlist.push(start_node);
    astar_map_[start_node.idx.x()][start_node.idx.y()] = start_node;
    double one_step = a_star_.simulation_step / a_star_.resolution_x;
    
    // Start planning
    while (!astar_openlist.empty() && completeness_flag == false)
    {
        A_Star_Node cur_node = astar_openlist.top();
        astar_openlist.pop();
        astar_map_[cur_node.idx.x()][cur_node.idx.y()].open = 0;
        astar_map_[cur_node.idx.x()][cur_node.idx.y()].close = 1;
        if (cur_node.h < closest_node.h)
        {
            closest_node = cur_node;
        }
        for (int i = 0; i < 8; i++)
        {
            A_Star_Node child_node;
            child_node.p = calc_grid_position(cur_node.idx) + motion[i] * a_star_.simulation_step;
            child_node.idx = calc_xy_index(child_node.p);
            child_node.parent_idx = cur_node.idx;
            child_node.g = cur_node.g + motion_length[i];
            child_node.h = calc_heuristic(child_node.idx, goal_node.idx);
            child_node.f = child_node.g + a_star_.multiplier_H_for_A_star * child_node.h;
            // If the child node has been explored ever before, and it has been closed:
            if (astar_map_[child_node.idx.x()][child_node.idx.y()].close == 1)
                continue;
            // If the previously found parent of the child is not good enough, a change is to be made
            if (astar_map_[child_node.idx.x()][child_node.idx.y()].open == 1)
            {
                if (astar_map_[child_node.idx.x()][child_node.idx.y()].f > child_node.f)
                {
                    astar_map_[child_node.idx.x()][child_node.idx.y()] = child_node;
                }
                continue;
            }
            // Now the child node is ensured to be newly expanded
            vector<math::Vec2d> Verify = extremum_node(child_node.idx);
            if (Verify[0].x() < 0 || (obj_map_[Verify[1].x()][Verify[1].y()][1] - obj_map_[Verify[1].x()][Verify[0].y() - 1][1] - obj_map_[Verify[0].x() - 1][Verify[1].y()][1] + obj_map_[Verify[0].x() - 1][Verify[0].y() - 1][1] > 0))
            {
                child_node.close = 1;
                astar_map_[child_node.idx.x()][child_node.idx.y()] = child_node;
                continue;
            }
            // Now the child node is both new and collision-free.
            child_node.open = 1;
            astar_openlist.push(child_node);
            astar_map_[child_node.idx.x()][child_node.idx.y()] = child_node;
            // If child node is the goal node
            if (child_node.idx.DistanceTo(goal_node.idx) < 2 * one_step)
            {
                closest_node = child_node;
                completeness_flag = true;
            }
        }
    }
    A_star_path path;
    A_Star_Node final_node = closest_node;
    path.path_length = 0;
    path.completeness_flag = completeness_flag;
    A_Star_Node next_node = astar_map_[final_node.parent_idx.x()][final_node.parent_idx.y()];
    while (!(next_node.idx == start_node.idx))
    {
        path.x.push_back(final_node.p.x());
        path.y.push_back(final_node.p.y());
        path.theta.push_back(0);
        //        path.theta.push_back(final_node.theta);
        path.path_length = path.path_length + sqrt(pow(final_node.p.x() - next_node.p.x(), 2) + pow(final_node.p.y() - next_node.p.y(), 2));
        final_node = next_node;
        next_node = astar_map_[final_node.parent_idx.x()][final_node.parent_idx.y()];
    }
    path.x.push_back(final_node.p.x());
    path.y.push_back(final_node.p.y());
    path.theta.push_back(0);
    //    path.theta.push_back(final_node.theta);
    path.path_length = path.path_length + sqrt(pow(final_node.p.x() - next_node.p.x(), 2) + pow(final_node.p.y() - next_node.p.y(), 2));
    path.x.push_back(next_node.p.x());
    path.y.push_back(next_node.p.y());
    path.theta.push_back(0);
    //    path.theta.push_back(next_node.theta);
    
    return path;
}
