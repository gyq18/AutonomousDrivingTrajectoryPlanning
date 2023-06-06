#include "Visualize.h"
#include "mathstruct.h"
#include "TransformP2T.h"
#include "vec2d.h"
using namespace cv;
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

void VisualizeStaticResults(struct Trajectory trajectory) {    
    double xmax = *max_element(trajectory.x.begin(), trajectory.x.end());
    double xmin = *min_element(trajectory.x.begin(), trajectory.x.end());
    double ymax = *max_element(trajectory.y.begin(), trajectory.y.end());
    double ymin = *min_element(trajectory.y.begin(), trajectory.y.end());
    cout << "max_x: " << xmax << endl;
    cout << "min_x: " << xmin << endl;
    cout << "max_y: " << ymax << endl;
    cout << "min_y: " << ymin << endl;
    cv_axis2d axis(-5, 65, -5, 65);

    Mat canvas = Mat::ones(Size(512, 512 * (axis.y_h - axis.y_l) / (axis.x_h - axis.x_l)), CV_8UC3);
    canvas.setTo(255);
    int nstep = trajectory.x.size();

    // draw obstacle
    if (Nobs > 0) {
        for (int j = 0; j < Nobs;++j) {
            vector<Point> pts = axis.plist_in_canvas(obstacles_[j],canvas);
            fillPoly(canvas, pts,Scalar(0.7451* 255, 0.7451* 255, 0.7451* 255));
        }
    }

    // draw path
    vector < Point> path_p = axis.plist_in_canvas(trajectory.x,trajectory.y,canvas);
    polylines(canvas, path_p, false, Scalar(39, 127,  255), 1, 8, 0);
    
    for (int i = 0;i < nstep;++i) {
    // draw vehicle
        vector<math::Vec2d> V = CreateVehiclePolygon(trajectory.x[i],trajectory.y[i], trajectory.theta[i]);
        vector<Point> V_p = axis.plist_in_canvas(V,canvas);
        polylines(canvas, V_p, true, Scalar(234,217,153), 1, 8, 0);
        circle(canvas, path_p[i],2, Scalar(39, 127,  255), -1);
    }

    // draw point which is start or end, red one is the end point
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[nstep -1], trajectory.y[nstep -1], canvas), 5,1);
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[0], trajectory.y[0],canvas),5);
    axis.DrawAxis(canvas);
    imshow("VisualizeStaticResults", canvas);
    waitKey();
};

// draw a frame in the i'th step
void drawframe(struct Trajectory trajectory,int i, cv_axis2d axis,Mat canvas) {
    int nstep = trajectory.x.size();
    if (Nobs > 0) {
        for (int j = 0; j < Nobs;++j) {
            vector<Point> pts = axis.plist_in_canvas(obstacles_[j], canvas);
            fillPoly(canvas, pts, Scalar(0.7451 * 255, 0.7451 * 255, 0.7451 * 255));
        }
    }
    // draw path
    vector < Point> path_p = axis.plist_in_canvas(trajectory.x, trajectory.y, canvas);
    polylines(canvas, path_p, false, Scalar(39, 127,  255), 1, 8, 0);
    for (int i = 0;i < nstep;++i) {
        circle(canvas, path_p[i], 2, Scalar(39, 127, 255), -1);
    }

    // draw point which is start or end, red one is the end point
    vector<math::Vec2d> V = CreateVehiclePolygon(trajectory.x[i], trajectory.y[i], trajectory.theta[i]);
    vector<Point> V_p = axis.plist_in_canvas(V, canvas);
    polylines(canvas, V_p, true, Scalar(234, 217, 153), 1, 8, 0);
    axis.DrawAxis(canvas);
}
void VisualizeDynamicResults(struct Trajectory trajectory) {
    int nstep = trajectory.x.size();
    double xmax = *max_element(trajectory.x.begin(), trajectory.x.end());//最大值
    double xmin = *min_element(trajectory.x.begin(), trajectory.x.end());//最小值
    double ymax = *max_element(trajectory.y.begin(), trajectory.y.end());//最大值
    double ymin = *min_element(trajectory.y.begin(), trajectory.y.end());//最小值
    cout << "max_x: " << xmax << endl;
    cout << "min_x: " << xmin << endl;
    cout << "max_y: " << ymax << endl;
    cout << "min_y: " << ymin << endl;
    cv_axis2d axis(-5, 65, -5, 65);
    Mat canvas = Mat::ones(Size(512, 512 * (axis.y_h - axis.y_l) / (axis.x_h - axis.x_l)), CV_8UC3);
    
    // draw frames
    for (int i = 0;i < nstep;++i) {        
        canvas.setTo(255);
        drawframe(trajectory,i, axis,canvas);
        imshow("VisualizeDynamicResults", canvas);
        waitKey(20); //20ms a frame
    }
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[nstep - 1], trajectory.y[nstep - 1], canvas), 5, 1);
    DrawCircle(canvas, axis.p_in_canvas(trajectory.x[0], trajectory.y[0], canvas), 5);
    imshow("VisualizeDynamicResults", canvas);
    waitKey();
};

void cv_axis2d::DrawAxis(Mat canvas) {
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
    vector<Point> xptr;
    xptr.push_back(Point(bs, x_axis_pos));
    xptr.push_back(Point(w - bs, x_axis_pos));
    polylines(canvas, xptr,false,axis_color);
    //Y axis
    vector<Point> yptr;
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