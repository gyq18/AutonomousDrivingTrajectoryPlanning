//  Check collision for one waypoint with all obstacles
//  CheckByOBB.hpp

#ifndef CheckByOBB_hpp
#define CheckByOBB_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// OBB projection quadrilateral collision detection
bool OBB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2);

// OBB collision detection
bool Vehicle_OBB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

bool CheckByOBB(double x, double y, double theta);

#endif /* CheckByOBB_hpp */
