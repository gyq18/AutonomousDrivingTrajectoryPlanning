/*
 * @Author: 李晟永 lishengy17@mails.tsinghua.edu.cn
 * @Date: 2022-05-30 10:12:11
 * @LastEditors: 李晟永 lishengy17@mails.tsinghua.edu.cn
 * @LastEditTime: 2022-05-30 14:12:07
 * @FilePath: /cpp code library/vis_api.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "vec2d.h"


using namespace cv;
class cv_axis2d {
public:
    // limit of axis in x & y
    double x_l;
    double x_h;
    double y_l;
    double y_h;
    cv_axis2d(const double xl, const double xh, const double yl, const double yh) : x_l(xl), x_h(xh), y_l(yl), y_h(yh) {}
    
    // Converts a point of the coordinate system to canvas coordinates
    Point p_in_canvas(double x, double y, Mat canvas){
        int w = canvas.cols;
        int h = canvas.rows;
        double px = w / (x_h - x_l) * (x - x_l);
        double py = h / (y_h - y_l) * (y_h - y );
        Point p_draw(px, py);
        return p_draw;
    }

    // Converts a point of the coordinate system to canvas coordinates
    Point p_in_canvas(math::Vec2d p, Mat canvas){
        return p_in_canvas(p.x(), p.y(), canvas);
    }

    // Converts the points of the coordinate system to canvas coordinates
    std::vector<Point> plist_in_canvas(std::vector<math::Vec2d> obs, Mat canvas){
        std::vector<Point> p_drawlist;
        int N = obs.size();
        for (int i = 0;i < N; ++i) {
            p_drawlist.push_back(p_in_canvas(obs[i], canvas));
        }
        return p_drawlist;
    }

    // Converts the points of the coordinate system to canvas coordinates
    std::vector<Point> plist_in_canvas(std::vector<double> X, std::vector<double> Y, Mat canvas) {
        std::vector<Point> p_drawlist;
        int N = X.size();
        for (int i = 0;i < N; ++i) {
            p_drawlist.push_back(p_in_canvas(X[i],Y[i], canvas));
        }
        return p_drawlist;
    }
    void DrawAxis(Mat canvas)
    {
        Scalar axis_color(0, 0, 0);

        int w = canvas.cols;
        int h = canvas.rows;

        // size of graph
        int bs = 10;
        int gh = h - bs * 2;
        int gw = w - bs * 2;

        // draw the horizontal and vertical axis
        // let x, y axies cross at zero if possible.
        double y_ref = this->y_l;

        int x_axis_pos = h - bs - cvRound((y_ref - this->y_l) * h/(this->y_h - this->y_l));
        //X axis
        std::vector<Point> xptr;
        xptr.push_back(Point(bs, x_axis_pos));
        xptr.push_back(Point(w - bs, x_axis_pos));
        polylines(canvas, xptr,false,axis_color);
        //Y axis
        std::vector<Point> yptr;
        yptr.push_back(Point(bs, h - bs));
        yptr.push_back(Point(bs, h - bs - gh));
        polylines(canvas, yptr, false, axis_color);


        // Write the scale of the y axis

        int chw = 6, chh = 10;
        char text[16];

        // y max
        if ((this->y_h - y_ref) > 0.05 * (this->y_h - this->y_l)) {
            snprintf(text, sizeof(text) - 1, "%.1f", this->y_h);
            putText(canvas, text, Point(bs, bs / 2), FONT_HERSHEY_PLAIN,1,axis_color);
        }
        // y min
        if ((y_ref - this->y_l) > 0.05 * (this->y_h - this->y_l)) {
            snprintf(text, sizeof(text) - 1, "%.1f", this->y_l);
            putText(canvas, text, Point(bs, h - bs / 2), FONT_HERSHEY_PLAIN, 1, axis_color);
        }


        double y_scale_pixes = h/5;
        //double y_scale_pixes = max(gh / ceil(y_max - y_min) / 100, chh * 2.0);
        for (int i = 1; i < ceil(gh / y_scale_pixes) + 1; i++) {
            snprintf(text, sizeof(text) - 1, "%.1f", this->y_l + i * y_scale_pixes / h * (this->y_h - this->y_l));
            putText(canvas, text, Point(bs / 5, h - i * y_scale_pixes), FONT_HERSHEY_PLAIN, 1, axis_color);
        }

        // x_max
        snprintf(text, sizeof(text) - 1, "%.3f", this->x_h);
        putText(canvas, text, Point(w - bs / 2 - strlen(text) * chw, x_axis_pos), FONT_HERSHEY_PLAIN, 1, axis_color);

        // x min
        snprintf(text, sizeof(text) - 1, "%.3f", this->x_l);
        putText(canvas, text, Point(bs, x_axis_pos), FONT_HERSHEY_PLAIN, 1, axis_color);

        //double x_scale_pixes = chw * 4;
        //double x_scale_pixes = min(chw * 4.0, ceil(x_max - x_min));
        double x_scale_pixes = w/5;   
        for (int i = 1; i < ceil(gw / x_scale_pixes) + 1; i++) {
            snprintf(text, sizeof(text) - 1, "%.1f", this->x_l + i * x_scale_pixes / w*(this->x_h - this->x_l));
            putText(canvas, text, Point(i * x_scale_pixes, x_axis_pos + chh), FONT_HERSHEY_PLAIN, 1, axis_color);

        }
    }
};

struct Trajectory_ {
	std::vector<double> x, y, theta, v, a, phi, omega;
};

extern "C" {
void visualize_static_results(double *x, double* y, double* theta, int len);
void visualize_dynamic_results(double *x, double* y, double* theta, int len);

}