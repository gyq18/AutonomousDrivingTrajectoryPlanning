// Check collision for one waypoint with all obstacles
//  Comparative_area.hpp

#ifndef CheckByArea_hpp
#define CheckByArea_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Collision detection of point p and obstacle V by comparative area method
bool P_Comparative_Area_Collision(math::Vec2d p, vector<math::Vec2d> V);

// Collision detection by comparative area method
// Algorithm reference https://zhuanlan.zhihu.com/p/449795053
bool Comparative_Area_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

bool CheckByArea(double x, double y, double theta);

#endif /* CheckByArea_hpp */
