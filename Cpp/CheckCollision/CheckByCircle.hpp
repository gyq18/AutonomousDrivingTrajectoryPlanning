// Check collision for one waypoint with all obstacles
// CheckByCircle.hpp

#ifndef CheckByCircle_hpp
#define CheckByCircle_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Whether the circle with (x, y) as the center and r as the radius intersects the line segment o1o2
bool Circumscribed_Circle(double x, double y, double r, math::Vec2d o1, math::Vec2d o2);

// Whether the circle with P as the center and r as the radius intersects with the line segment o1o2
bool Circumscribed_Circle(math::Vec2d p, double r, math::Vec2d o1, math::Vec2d o2);

// Whether the detection point is in the obstacle
bool check_in(math::Vec2d p, vector<math::Vec2d> obj);

// Whether the front and rear circumscribed circles of the vehicle collide with obstacles
bool Vehicle_Circumscribed_Circle_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj);

bool CheckByCircle(double x, double y, double theta);

#endif /* CheckByCircle_hpp */
