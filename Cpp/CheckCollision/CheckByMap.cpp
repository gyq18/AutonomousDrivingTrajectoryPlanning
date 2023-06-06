//  CheckByMap.cpp

#include "CheckByMap.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include "mathstruct.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

extern vector<vector<math::Vec2d>> obstacles_;
extern int Nobs;
extern vector<vector<int>> costmap_;
extern Vehicle_geometrics_ vehicle_geometrics_;
extern struct Hybrid_astar_ hybrid_astar_;

using namespace cv;

bool CheckByMap(double x, double y, double theta)
{
    if (costmap_.empty())
        CreateDilatedCostmap();
    // The center of the front circle of the vehicle is p1, the center of the rear circle is p2, and the radius of the circumscribed circle is r
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    // Get vehicle Edge Data
    double p1x = x + (vehicle_geometrics_.vehicle_length * 3 / 4 -
                      vehicle_geometrics_.vehicle_rear_hang) *
                         cos_theta;
    double p1y = y + (vehicle_geometrics_.vehicle_length * 3 / 4 -
                      vehicle_geometrics_.vehicle_rear_hang) *
                         sin_theta;

    double p2x = x + (vehicle_geometrics_.vehicle_length / 4 -
                      vehicle_geometrics_.vehicle_rear_hang) *
                         cos_theta;
    double p2y = y + (vehicle_geometrics_.vehicle_length / 4 -
                      vehicle_geometrics_.vehicle_rear_hang) *
                         sin_theta;

    math::Vec2d p1 = ConvertXYToIndex(p1x, p1y);
    math::Vec2d p2 = ConvertXYToIndex(p2x, p2y);

    if (costmap_[p1.x()][p1.y()] + costmap_[p2.x()][p2.y()] > 0)
        return true;
    return false;
}

void CreateDilatedCostmap()
{
    double minx;
    double miny;
    double maxx;
    double maxy;
    vector<vector<float>> costmap(hybrid_astar_.num_nodes_x, vector<float>(hybrid_astar_.num_nodes_y));
    vector<vector<int>> tmpmap(hybrid_astar_.num_nodes_x, vector<int>(hybrid_astar_.num_nodes_y));
    costmap_ = tmpmap;
    math::Vec2d id;
    for (int i = 0; i < obstacles_.size(); i++)
    {
        vector<math::Vec2d> obj;
        id = calc_xy_index(obstacles_[i][0]);
        minx = id.x();
        miny = id.y();
        maxx = id.x();
        maxy = id.y();
        obj.push_back(id);
        for (int j = 1; j < 4; j++)
        {
            id = calc_xy_index(obstacles_[i][j]);
            obj.push_back(id);
            if (minx > id.x())
                minx = id.x();
            if (maxx < id.x())
                maxx = id.x();
            if (miny > id.y())
                miny = id.y();
            if (maxy < id.y())
                maxy = id.y();
        }
        // Set the position of the obstacle to 1
        for (int m = minx; m < maxx + 1; m++)
        {
            for (int n = miny; n < maxy + 1; n++)
            {
                if (checkObj_point(m, n, obj))
                    costmap[m][n] = 100;
            }
        }
    }
    // Set border to 1
    for (int i = 0; i < hybrid_astar_.num_nodes_x; i++)
    {
        costmap[i][0] = 250;
        costmap[i][hybrid_astar_.num_nodes_y - 1] = 250;
    }
    for (int j = 0; j < hybrid_astar_.num_nodes_y; j++)
    {
        costmap[0][j] = 250;
        costmap[hybrid_astar_.num_nodes_x - 1][j] = 250;
    }
    Mat Tmp_map(0, costmap[0].size(), CV_32F);
    for (int i = 0; i < costmap.size(); ++i)
    {
        Mat Sample(1, costmap[0].size(), CV_32F, costmap[i].data());
        Tmp_map.push_back(Sample);
    }

    Mat Image_map;
    double length_unit = 0.5 * (hybrid_astar_.resolution_x + hybrid_astar_.resolution_y);
    double r = ceil(vehicle_geometrics_.radius / length_unit) + 2;
    Mat basic_elem = getStructuringElement(MORPH_ELLIPSE, Size(r, r));
    dilate(Tmp_map, Image_map, basic_elem);

    for (int x = 0; x < Image_map.rows; x++)
    {
        for (int y = 0; y < Image_map.cols; y++)
        {
            if (Image_map.at<float>(x, y) > 0)
                costmap_[x][y] = 1;
            else
                costmap_[x][y] = 0;
        }
    }
}
