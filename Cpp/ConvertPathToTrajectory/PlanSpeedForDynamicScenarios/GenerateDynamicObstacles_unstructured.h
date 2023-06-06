/*
 * @Author: lishengyong
 * @Date: 2022-05-15 14:04:04
 * @LastEditors: lishengyong 
 * @LastEditTime: 2022-05-15 15:55:27
 * @FilePath: /cpp code/GenerateDynamicObstacles_unstructured.h

 */
#pragma once

#include <vector>
#include "Eigen/Dense"

using Eigen::Array4d;

class DObstacles 
{

public:
    Array4d x;
    Array4d y;

public:
    DObstacles();
    DObstacles(Array4d x_list, Array4d y_list);

};

std::vector<std::vector<DObstacles>> GenerateDynamicObstacles_unstructured(void);


