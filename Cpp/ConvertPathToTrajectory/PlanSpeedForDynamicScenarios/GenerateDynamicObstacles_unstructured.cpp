
#include "GenerateDynamicObstacles_unstructured.h"
#include "global_structs.h"
#include <vector>
#include <assert.h>
#include <iostream>

using namespace std;

DObstacles::DObstacles(Array4d x_list, Array4d y_list)
{
    this->x = x_list;
    this->y = y_list;
}


vector<vector<DObstacles>> GenerateDynamicObstacles_unstructured(void) 
{
    extern StGraphSearch st_graph_search_;

    int Nobs_dyn = 5;

    vector<DObstacles> start_obs;
    vector<DObstacles> end_obs;
    
    start_obs.push_back(DObstacles({10, 15,15,10}, {10,10, 15,15}));
    end_obs.push_back(DObstacles({45, 50, 50, 45}, {10, 10, 15, 15}));

    start_obs.push_back(DObstacles({45, 50, 50, 45}, {10, 10, 15, 15}));
    end_obs.push_back(DObstacles({45, 50, 50, 45}, {45, 45, 50, 50}));

    start_obs.push_back(DObstacles({45, 50, 50, 45}, {45, 45, 50, 50}));
    end_obs.push_back(DObstacles({10, 15, 15, 10}, {45, 45, 50, 50}));

    start_obs.push_back(DObstacles({10, 15,15,10}, {45, 45, 50, 50}));
    end_obs.push_back(DObstacles({10, 15, 15, 10}, {10, 10, 15, 15}));

    start_obs.push_back(DObstacles({17, 19, 25, 23}, {19, 17, 23, 25}));
    end_obs.push_back(DObstacles({34, 36, 42, 40}, {36, 34, 40, 42}));


    // different from matlab code, we use size as (Nobs, num_node) for num_node is always largger
    vector<vector<DObstacles>> obstacle_map;
    for(int i = 0; i < Nobs_dyn; i++)
    {
        auto start_x = start_obs[i].x;
        auto start_y = start_obs[i].y;

        // use eval to avid aliasing.
        // I don't know why there is aliasing.
        // I think this may be caused by the lazy evaluation
        // x() returns an temp object and save it in stack/registor 
        // the evaluation is not performed immediately
        // when calling y() the compiler reuse the memory of 
        // temp object of x but it has not been used!
        // This caused the result returned by x() is covered by y().

        // finally I use direct *.x to avoid the creation of temp object.
        auto dx = (end_obs[i].x - start_x);
        auto dy = (end_obs[i].y - start_y);

        obstacle_map.push_back({});

        int node_num = st_graph_search_.num_nodes_t;
        for (int j = 0; j < node_num; j++) 
        {
            obstacle_map[i].push_back(
                DObstacles(
                    (start_x + dx / node_num * j).eval(), 
                    (start_y + dy / node_num * j).eval()
                )
            );

        }
    }

    return obstacle_map;
}