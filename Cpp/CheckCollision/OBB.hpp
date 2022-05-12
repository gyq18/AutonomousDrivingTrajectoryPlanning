//
//  OBB.hpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/16.
//

#ifndef OBB_hpp
#define OBB_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// OBB projection quadrilateral collision detection
// OBB投影四边形碰撞检测
bool OBB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2);

// OBB collision detection
// OBB碰撞检测
bool Vehicle_OBB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);


#endif /* OBB_hpp */
