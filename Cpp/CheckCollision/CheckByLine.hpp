//  CheckByLine.hpp

#ifndef CheckByLine_hpp
#define CheckByLine_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include "mathstruct.h"
#include <vector>
using namespace std;

// Collision detection by checking for lines and vertices of the gnerated rectangle (vehicle)
bool CheckByLine(double x, double y, double theta);

#endif /* CheckByLine_hpp */
