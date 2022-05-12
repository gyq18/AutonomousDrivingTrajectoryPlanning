//
//  Comparative_area.hpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/19.
//

#ifndef Comparative_area_hpp
#define Comparative_area_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Collision detection of point p and obstacle V by comparative area method
// p点与障碍物V的比较面积法碰撞检测
bool P_Comparative_Area_Collision(math::Vec2d p, vector<math::Vec2d> V);

// Collision detection by comparative area method
// Algorithm reference https://zhuanlan.zhihu.com/p/449795053
// 比较面积法碰撞检测
// 算法参考https://zhuanlan.zhihu.com/p/449795053
bool Comparative_Area_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

#endif /* Comparative_area_hpp */
