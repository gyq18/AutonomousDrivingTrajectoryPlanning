/*
 * @Author: 李晟永 lishengy17@mails.tsinghua.edu.cn
 * @Date: 2022-05-30 10:15:06
 * @LastEditors: 李晟永 lishengy17@mails.tsinghua.edu.cn
 * @LastEditTime: 2022-05-30 14:21:46
 * @FilePath: /cpp code library/vis_api.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "vis_api.h"
#include <vector>
#include <iostream>
using namespace std;

void DrawCircle(Mat img,Point place, int radius=2,int t=0)
{   
    if (t == 0) {
        circle(img, place, radius, Scalar(14, 201, 255), -1);
    }
    else if (t == 1) {
        circle(img, place, radius, Scalar( 1,1, 255), -1);
    }
}

void drawframe(struct Trajectory_ trajectory,int i, cv_axis2d axis,Mat canvas) {

    int nstep = trajectory.x.size();
    // if (Nobs > 0) {
    //     for (int j = 0; j < Nobs;++j) {
    //         vector<Point> pts = axis.plist_in_canvas(obstacles_[j], canvas);
    //         fillPoly(canvas, pts, Scalar(0.7451 * 255, 0.7451 * 255, 0.7451 * 255));
    //     }
    // }
    // draw path
    vector < Point> path_p = axis.plist_in_canvas(trajectory.x, trajectory.y, canvas);
    polylines(canvas, path_p, false, Scalar(39, 127,  255), 1, 8, 0);
    for (int i = 0;i < nstep;++i) {
        circle(canvas, path_p[i], 2, Scalar(39, 127, 255), -1);
    }

    // // draw point which is start or end, red one is the end point
    // vector<math::Vec2d> V = CreateVehiclePolygon(trajectory.x[i], trajectory.y[i], trajectory.theta[i]);
    // vector<Point> V_p = axis.plist_in_canvas(V, canvas);
    // polylines(canvas, V_p, true, Scalar(234, 217, 153), 1, 8, 0);
    axis.DrawAxis(canvas);
}

extern "C" {
void visualize_static_results(double *x, double* y, double* theta, int len)
{
    Trajectory_ trajectory; 
    trajectory.x = std::vector(x, x + len);
    trajectory.y = std::vector(y, y+len);
    trajectory.theta = std::vector(theta, theta + len);
    
    double xmax = *max_element(trajectory.x.begin(), trajectory.x.end());
    double xmin = *min_element(trajectory.x.begin(), trajectory.x.end());
    double ymax = *max_element(trajectory.y.begin(), trajectory.y.end());
    double ymin = *min_element(trajectory.y.begin(), trajectory.y.end());
    cout << "max_x: " << xmax << endl;
    cout << "min_x: " << xmin << endl;
    cout << "max_y: " << ymax << endl;
    cout << "min_y: " << ymin << endl;
    cv_axis2d axis(xmin-5, xmax+5, ymin-5, ymax+5);

    Mat canvas = Mat::ones(Size(512, 512 * (axis.y_h - axis.y_l) / (axis.x_h - axis.x_l)), CV_8UC3);
    canvas.setTo(255);
    int nstep = trajectory.x.size();

    // // draw obstacle
    // if (n_obs > 0) {
    //     for (int j = 0; j < Nobs;++j) {
    //         vector<Point> pts = axis.plist_in_canvas(obstacles_[j],canvas);
    //         fillPoly(canvas, pts,Scalar(0.7451* 255, 0.7451* 255, 0.7451* 255));
    //     }
    // }

    // draw path
    vector < Point> path_p = axis.plist_in_canvas(trajectory.x,trajectory.y,canvas);
    polylines(canvas, path_p, false, Scalar(39, 127,  255), 1, 8, 0);
    
    // for (int i = 0;i < nstep;++i) {
    // // draw vehicle
    //     vector<math::Vec2d> V = CreateVehiclePolygon(trajectory.x[i],trajectory.y[i], trajectory.theta[i]);
    //     vector<Point> V_p = axis.plist_in_canvas(V,canvas);
    //     polylines(canvas, V_p, true, Scalar(234,217,153), 1, 8, 0);
    //     circle(canvas, path_p[i],2, Scalar(39, 127,  255), -1);
    // }

    // draw point which is start or end, red one is the end point
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[nstep -1], trajectory.y[nstep -1], canvas), 5,1);
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[0], trajectory.y[0],canvas), 5, 0);
    axis.DrawAxis(canvas);
    imshow("VisualizeStaticResults", canvas);
    waitKey();

}

void visualize_dynamic_results(double *x, double* y, double* theta, int len)
{
    Trajectory_ trajectory;
    trajectory.x = std::vector(x, x + len);
    trajectory.y = std::vector(y, y+len);
    trajectory.theta = std::vector(theta, theta + len);
    
    int nstep = trajectory.x.size();
    double xmax = *max_element(trajectory.x.begin(), trajectory.x.end());//最大值
    double xmin = *min_element(trajectory.x.begin(), trajectory.x.end());//最小值
    double ymax = *max_element(trajectory.y.begin(), trajectory.y.end());//最大值
    double ymin = *min_element(trajectory.y.begin(), trajectory.y.end());//最小值
    cout << "max_x: " << xmax << endl;
    cout << "min_x: " << xmin << endl;
    cout << "max_y: " << ymax << endl;
    cout << "min_y: " << ymin << endl;
    cv_axis2d axis(xmin - 5, xmax + 5, ymin - 5, ymax + 5);
    Mat canvas = Mat::ones(Size(512, 512 * (axis.y_h - axis.y_l) / (axis.x_h - axis.x_l)), CV_8UC3);
    
    // draw frames
    for (int i = 0;i < nstep;++i) {        
        canvas.setTo(255);
        drawframe(trajectory,i, axis,canvas);
        imshow("VisualizeDynamicResults", canvas);
        waitKey(20); //20ms a frame
    }
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[nstep - 1], trajectory.y[nstep - 1], canvas), 5, 1);
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[0], trajectory.y[0], canvas), 5, 0);
    imshow("VisualizeDynamicResults", canvas);
    waitKey();

}

}