// Check collision for one waypoint with all obstacles
//  AABB.hpp

#ifndef CheckByAABB_hpp
#define CheckByAABB_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Calculates a set of non adjacent points of a projected quadrilateral
math::Vec2d Min(vector<math::Vec2d> V);
math::Vec2d Max(vector<math::Vec2d> V);

// AABB projection quadrilateral collision detection
bool AABB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2);

// AABB collision detection
bool Vehicle_AABB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

bool CheckByAABB(double x, double y, double theta);

#endif /* CheckByAABB_hpp */
