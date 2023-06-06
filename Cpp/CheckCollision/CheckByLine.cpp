//  CheckByLine.cpp

#include "CheckByLine.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

extern vector<vector<math::Vec2d>> obstacles_;
extern int Nobs;

// Collision detection by checking for lines and vertices of the gnerated rectangle (vehicle)
bool CheckByLine(double x, double y, double theta)
{
    vector<math::Vec2d> V = CreateVehiclePolygon(x, y, theta);
    int flag[4];
    for (int i = 0; i < Nobs; i++)
    {
        flag[0] = checkObj_linev(V[0], V[1], obstacles_[i]);
        flag[1] = checkObj_linev(V[1], V[2], obstacles_[i]);
        flag[2] = checkObj_linev(V[2], V[3], obstacles_[i]);
        flag[3] = checkObj_linev(V[3], V[0], obstacles_[i]);
    }
    if (flag[0] + flag[1] + flag[2] + flag[3] > 0)
    {
        return true;
    }
    return false;
}
