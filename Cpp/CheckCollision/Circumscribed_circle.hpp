//
//  Circumscribed_circle.hpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/15.
//

#ifndef Circumscribed_circle_hpp
#define Circumscribed_circle_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

//Whether the circle with (x, y) as the center and r as the radius intersects the line segment o1o2
// 以(x, y)为圆心，r为半径的圆是否与线段o1o2相交
bool Circumscribed_Circle(double x, double y, double r, math::Vec2d o1, math::Vec2d o2);

//Whether the circle with P as the center and r as the radius intersects with the line segment o1o2
// 以p为圆心，r为半径的圆是否与线段o1o2相交
bool Circumscribed_Circle(math::Vec2d p, double r, math::Vec2d o1, math::Vec2d o2);

//Whether the front and rear circumscribed circles of the vehicle collide with obstacles
// 车辆前后两个外接圆是否与障碍物有碰撞
bool Vehicle_Circumscribed_Circle_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

#endif /* Circumscribed_circle_hpp */
