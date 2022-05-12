//
//  Circumscribed_circle.cpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/15.
//

#include "Circumscribed_circle.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

//Whether the circle with (x, y) as the center and r as the radius intersects the line segment o1o2
// 以(x, y)为圆心，r为半径的圆是否与线段o1o2相交
bool Circumscribed_Circle(double x, double y, double r, math::Vec2d o1, math::Vec2d o2)
{
    math::Vec2d p(x, y);
    return Circumscribed_Circle(p, r, o1, o2);
}

//Whether the circle with P as the center and r as the radius intersects with the line segment o1o2
// 以p为圆心，r为半径的圆是否与线段o1o2相交
bool Circumscribed_Circle(math::Vec2d p, double r, math::Vec2d o1, math::Vec2d o2)
{
    //First, determine whether o1o2 is within the circumscribed circle
    // 首先确定o1o2是否在外接圆内
    if (p.DistanceTo(o1) <= r)
        return true;
    if (p.DistanceTo(o2) <= r)
        return true;
    //If o1o2 they are all outside the obstacle, determine whether the circle intersects the line segment
    // 如果o1o2都在障碍物外，则确定圆与线段是否相交
    double norm_dist = (o2 - o1).Length();
    if (norm_dist < 1e-06)
    {
        return false;
    }
    else
    {
        double A = -(o2.y() - o1.y());
        double B = o2.x() - o1.x();
        //Distance from circle center to line segment
        //圆心到线段距离
        double dist = abs(A * (p.x() - o1.x()) + B * (p.y() - o1.y())) / sqrt(pow(A, 2) + pow(B, 2));
        if (dist > r)
        {
            return false;
        }
        else
        {
            double m = B * p.x() - A * p.y();
            //Calculate the coordinates of the intersection of the vertical line of the center of the circle and the straight line o1o2
            //求得圆心垂线到直线o1o2的交点坐标
            double x4 = (pow(A, 2) * o1.x() - B * (m - o1.y() * A)) / (pow(A, 2) + pow(B, 2));
            if(abs(B)>1e-06)
            {
                if ((o1.x() - x4) * (o2.x() - x4) < 0)
                    return true;
            }
            else
            {
                double y4 = (B*x4+m)/A;
                if((o1.y() - y4) * (o2.y() - y4) < 0)
                    return true;
            }
        }
    }
    return false;
}

//Whether the front and rear circumscribed circles of the vehicle collide with obstacles
// 车辆前后两个外接圆是否与障碍物有碰撞
bool Vehicle_Circumscribed_Circle_Collision(math::Vec2d p, double theta,  vector<math::Vec2d> obj)
{
    int ncorner = 4;
    bool Is_Collision = false;
    //The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    // 车辆前圆圆心为p1，后圆圆心为p2,外接圆半径为r
    double p1x, p1y, p2x, p2y;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double r = sqrt(pow(vehicle_geometrics_.vehicle_length/4,2)+pow(vehicle_geometrics_.vehicle_width/2,2));
    
    p1x = p.x() + (vehicle_geometrics_.vehicle_length * 3 / 4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p1y = p.y() + (vehicle_geometrics_.vehicle_length * 3 / 4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;
    
    p2x = p.x() + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * cos_theta;
    p2y = p.y() + (vehicle_geometrics_.vehicle_length / 4 - vehicle_geometrics_.vehicle_rear_hang) * sin_theta;
    
    math::Vec2d p1(p1x, p1y);
    math::Vec2d p2(p2x, p2y);
    
    for (int i = 0; i < ncorner; i++)
    {
        Is_Collision = Circumscribed_Circle(p1, r, obj[i],obj[(i+1)%ncorner]);
        if(Is_Collision==true)
            return Is_Collision;
    }
    for (int i = 0; i < ncorner; i++)
    {
        Is_Collision = Circumscribed_Circle(p2, r, obj[i],obj[(i+1)%ncorner]);
        if(Is_Collision==true)
            return Is_Collision;
    }
    return Is_Collision;
    
}
