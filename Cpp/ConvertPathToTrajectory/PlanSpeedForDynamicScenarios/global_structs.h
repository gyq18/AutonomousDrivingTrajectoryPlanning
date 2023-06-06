/*
 * @Author: lishengyong
 * @Date: 2022-05-15 15:16:02
 * @LastEditors: lishengyong
 * @LastEditTime: 2022-05-15 15:59:27
 * @FilePath: /cpp code/StGraphSearch.h

 */



#pragma once

class StGraphSearch 
{
public: 
    int k;
    int num_nodes_s;
    int num_nodes_t;
    double multiplier_H_for_A_star;
    double penalty_for_inf_velocity;
    double max_t;
    double max_s;

    double resolution_s;
    double resolution_t;
};
