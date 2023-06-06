#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include <opencv2/opencv.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "bezier.h"
using namespace std;
using namespace cv;

// visualize planning results
void VisualizeStaticResults(struct Trajectory trajectory);
void VisualizeStaticResults(struct Trajectory trajectory, vector<Bezier::bounding_box> box_list);
// visualize planning results Dynamically
void VisualizeDynamicResults(struct Trajectory trajectory);
extern vector < vector<math::Vec2d>> obstacles_;

//Converts the points of the coordinate system to canvas coordinates
class cv_axis2d {
public:
    // limit of axis in x & y
    double x_l;
    double x_h;
    double y_l;
    double y_h;
    cv_axis2d(const double xl, const double xh, const double yl, const double yh) : x_l(xl), x_h(xh), y_l(yl), y_h(yh) {}

    // Converts a point of the coordinate system to canvas coordinates
    Point p_in_canvas(double x, double y, Mat canvas) {
        int w = canvas.cols;
        int h = canvas.rows;
        double px = w / (x_h - x_l) * (x - x_l);
        double py = h / (y_h - y_l) * (y_h - y);
        Point p_draw(px, py);
        return p_draw;
    }

    // Converts a point of the coordinate system to canvas coordinates
    Point p_in_canvas(math::Vec2d p, Mat canvas) {
        return p_in_canvas(p.x(), p.y(), canvas);
    }

    // Converts the points of the coordinate system to canvas coordinates
    vector<Point> plist_in_canvas(vector<math::Vec2d> obs, Mat canvas) {
        vector<Point> p_drawlist;
        int N = obs.size();
        for (int i = 0; i < N; ++i) {
            p_drawlist.push_back(p_in_canvas(obs[i], canvas));
        }
        return p_drawlist;
    }

    // Converts the points of the coordinate system to canvas coordinates
    vector<Point> plist_in_canvas(vector<double> X, vector<double> Y, Mat canvas) {
        vector<Point> p_drawlist;
        int N = X.size();
        for (int i = 0; i < N; ++i) {
            p_drawlist.push_back(p_in_canvas(X[i], Y[i], canvas));
        }
        return p_drawlist;
    }
    void DrawAxis(Mat canvas);
};