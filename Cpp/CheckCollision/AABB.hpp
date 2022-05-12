//
//  AABB.hpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/16.
//

#ifndef AABB_hpp
#define AABB_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Calculates a set of non adjacent points of a projected quadrilateral
// 计算投影四边形的一组不相邻的点
math::Vec2d Min(vector<math::Vec2d> V);
math::Vec2d Max(vector<math::Vec2d> V);

// AABB projection quadrilateral collision detection
// AABB投影四边形碰撞检测
bool AABB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2);

// AABB collision detection
// AABB碰撞检测
bool Vehicle_AABB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

#endif /* AABB_hpp */
