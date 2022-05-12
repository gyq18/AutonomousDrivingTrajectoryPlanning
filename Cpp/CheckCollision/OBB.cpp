//
//  OBB.cpp
//  untitled
//
//  Created by 枉叹之 on 2022/4/16.
//

// OBB算法有参考博文https://blog.csdn.net/silangquan/article/details/50812425

#include "OBB.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

// OBB projection quadrilateral collision detection
// OBB投影四边形碰撞检测
bool OBB_Quadrilateral_Collision(vector<math::Vec2d> V1, vector<math::Vec2d> V2)
{
    // coordinate position of the center of rectangle A
    math::Vec2d PA = (V1[0] + V1[1] + V1[2] + V1[3]) / 4;
    // half width of A (corresponds with the local x-axis of A)
    double WA = (V1[3] - V1[0]).Length() / 2;
    // half height of A (corresponds with the local y-axis of A)
    double HA = (V1[1] - V1[0]).Length() / 2;
    // unit vector representing the local x-axis of A
    math::Vec2d Ax = (V1[3] - V1[0]) / (2 * WA);
    // unit vector representing the local y-axis of A
    math::Vec2d Ay = (V1[1] - V1[0]) / (2 * HA);
    
    // coordinate position of the center of rectangle B
    math::Vec2d PB = (V2[0] + V2[1] + V2[2] + V2[3]) / 4;
    // half width of A (corresponds with the local x-axis of B)
    double WB = (V2[3] - V2[0]).Length()/2;
    // half height of A (corresponds with the local y-axis of B)
    double HB = (V2[1] - V2[0]).Length()/2;
    // unit vector representing the local x-axis of B
    math::Vec2d Bx = (V2[3] - V2[0]) / (2 * WB);
    // unit vector representing the local y-axis of B
    math::Vec2d By = (V2[1] - V2[0]) / (2 * HB);
    
    math::Vec2d T = PB - PA;
    
    // Separating Axis Judge
    // 分离轴判断
    int Separating_Axis = 0;
    
    // CASE 1:
    // L = Ax
    // | T • Ax | > WA + | ( WB*Bx ) • Ax | + |( HB*By ) • Ax |
    // If true, there is a separating axis parallel Ax.
    if (abs(T.InnerProd(Ax)) > WA + abs((WB * Bx).InnerProd(Ax)) + abs((HB * By).InnerProd(Ax)))
        Separating_Axis++;
    
    // CASE 2:
    // L = Ay
    // | T • Ay | > HA + | ( WB*Bx ) • Ay | + |( HB*By ) • Ay |
    // If true, there is a separating axis parallel Ax.
    if (abs(T.InnerProd(Ay)) > HA + abs((WB * Bx).InnerProd(Ay)) + abs((HB * By).InnerProd(Ay)))
        Separating_Axis++;
    
    // CASE 3:
    // L = Bx
    // | T • Bx | > | ( WA* Ax ) • Bx | + | ( HA*Ay ) • Bx | + WB
    // If true, there is a separating axis parallel Bx.
    if (abs(T.InnerProd(Bx)) > WB + abs((WA * Ax).InnerProd(Bx)) + abs((HA * Ay).InnerProd(Bx)))
        Separating_Axis++;
    
    // CASE 4:
    // L = By
    // | T • By | > | ( WA* Ax ) • By | + | ( HA*Ay ) • By | + HB
    // If true, there is a separating axis parallel By.
    if (abs(T.InnerProd(By)) > HB + abs((WA * Ax).InnerProd(By)) + abs((HA * Ay).InnerProd(By)))
        Separating_Axis++;
    
    if (Separating_Axis > 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// OBB collision detection
// OBB碰撞检测
bool Vehicle_OBB_Collision(math::Vec2d p, double theta, vector<math::Vec2d> obj)
{
    // Get vehicle Edge Data
    //获取车辆边缘数据
    vector<math::Vec2d> V = CreateVehiclePolygon(p.x(), p.y(), theta);
    
    return OBB_Quadrilateral_Collision(V, obj);
}
