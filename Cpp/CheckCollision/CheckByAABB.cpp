// Check collision for one waypoint with all obstacles
// CheckByAABB.cpp

#include "CheckByAABB.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

extern vector<vector<math::Vec2d>> obstacles_;
extern int Nobs;

// Calculates a set of non adjacent points of a projected quadrilateral
math::Vec2d Min(vector<math::Vec2d> V)
{
    double minx = V[0].x();
    double miny = V[0].y();
    for (int i = 1; i < 4; i++)
    {
        if (V[i].x() < minx)
            minx = V[i].x();
        if (V[i].y() < miny)
            miny = V[i].y();
    }
    math::Vec2d m(minx, miny);
    return m;
}
math::Vec2d Max(vector<math::Vec2d> V)
{
    double maxx = V[0].x();
    double maxy = V[0].y();
    for (int i = 1; i < 4; i++)
    {
        if (V[i].x() > maxx)
            maxx = V[i].x();
        if (V[i].y() > maxy)
            maxy = V[i].y();
    }
    math::Vec2d m(maxx, maxy);
    return m;
}

// AABB projection quadrilateral collision detection
bool AABB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2)
{
    int IsCollisionx = 0;
    int IsCollisiony = 0;
    for (int i = 0; i < 2; i++)
    {
        // X-axis
        if ((V1[i].x() - V2[0].x()) * (V1[i].x() - V2[1].x()) <= 0)
            IsCollisionx = 1;
        // Y-axis
        if ((V1[i].y() - V2[0].y()) * (V1[i].y() - V2[1].y()) <= 0)
            IsCollisiony = 1;
    }
    for (int i = 0; i < 2; i++)
    {
        if ((V2[i].x() - V1[0].x()) * (V2[i].x() - V1[1].x()) <= 0)
            IsCollisionx = 1;
        if ((V2[i].y() - V1[0].y()) * (V2[i].y() - V1[1].y()) <= 0)
            IsCollisiony = 1;
    }
    if (IsCollisionx == 1 && IsCollisiony == 1)
        return true;
    else
        return false;
}

// AABB collision detection
bool Vehicle_AABB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj)
{
    // Get vehicle Edge Data
    vector<math::Vec2d> V = CreateVehiclePolygon(p.x(), p.y(), theta);
    // Set vehicle projection data
    math::Vec2d A, B;
    A = Min(V);
    B = Max(V);
    vector<math::Vec2d> V1;
    V1.push_back(A);
    V1.push_back(B);

    // Set obj projection data
    math::Vec2d C, D;
    C = Min(obj);
    D = Max(obj);
    vector<math::Vec2d> V2;
    V2.push_back(C);
    V2.push_back(D);
    // Judge whether there is collision between projections
    return AABB_Quadrilateral_Collision(V1, V2);
}

bool CheckByAABB(double x, double y, double theta)
{
    math::Vec2d p(x, y);
    for (int i = 0; i < Nobs; i++)
    {
        if (Vehicle_AABB_Collision(p, theta, obstacles_[i]))
            return true;
    }
    return false;
}
