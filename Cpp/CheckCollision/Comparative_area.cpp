//
//  Comparative_area.cpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/19.
//

#include "Comparative_area.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

// Collision detection of point p and obstacle V by comparative area method
// p点与障碍物V的比较面积法碰撞检测
bool P_Comparative_Area_Collision(math::Vec2d p, vector<math::Vec2d> obj)
{
    int ncorner = 4;
    double obj_area = 0;
    for (int i = 0; i < ncorner; i = i + 2)
    {
        obj_area = obj_area + triArea(obj[i % ncorner], obj[(i + 1) % ncorner], obj[(i + 2) % ncorner]);
    }
    double p_area = 0;
    for (int i = 0; i < ncorner; i++)
    {
        p_area = p_area + triArea(p, obj[i], obj[(i + 1) % ncorner]);
    }
//    cout<<endl<<p_area<<" "<<obj_area<<endl;
    if (p_area > obj_area)
        return false;
    else
        return true;
}

// Collision detection by comparative area method
// Algorithm reference https://zhuanlan.zhihu.com/p/449795053
// 比较面积法碰撞检测
// 算法参考https://zhuanlan.zhihu.com/p/449795053
bool Comparative_Area_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj)
{
    // Get vehicle Edge Data
    //获取车辆边缘数据
    vector<math::Vec2d> V = CreateVehiclePolygon(p.x(), p.y(), theta);
    bool Is_Collision = false;
    for (int i = 0; i < 4; i++)
    {
        Is_Collision = P_Comparative_Area_Collision(V[i], obj);
        if (Is_Collision == true)
            return true;
    }
    for (int i = 0; i < 4; i++)
    {
        Is_Collision = P_Comparative_Area_Collision(obj[i], V);
        if (Is_Collision == true)
            return true;
    }
    return false;
}
