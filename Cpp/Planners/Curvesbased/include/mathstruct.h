#pragma once
#include <iostream>
#include <math.h>
#include "vec2d.h"
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
#define M_PI       3.14159265358979323846   // pistruct hybrid_astar_
//vehicle geometrics settings
struct Vehicle_geometrics_ {
    double vehicle_wheelbase;  // L_W,wheelbase of the ego vehicle (m)
    double vehicle_front_hang; // L_F,front hang length of the ego vehicle (m)
    double vehicle_rear_hang;// L_R,rear hang length of the ego vehicle (m)
    double vehicle_width; // width of the ego vehicle (m)
    double vehicle_length; // length of the ego vehicle (m)
    double radius, r2x, f2x;

    Vehicle_geometrics_(double wheelbase = 2.8, double front_hang = 0.96, double rear_hang = 0.929, double width = 1.942) {
        this->vehicle_wheelbase = wheelbase;  // L_W,wheelbase of the ego vehicle (m)
        this->vehicle_front_hang = front_hang; // L_F,front hang length of the ego vehicle (m)
        this->vehicle_rear_hang = rear_hang;// L_R,rear hang length of the ego vehicle (m)
        this->vehicle_width = width; // width of the ego vehicle (m)
        this->vehicle_length = this->vehicle_wheelbase + this->vehicle_front_hang + this->vehicle_rear_hang;// length of the ego vehicle (m)
        this->radius = hypot(0.25 * this->vehicle_length, 0.5 * this->vehicle_width);
        this->r2x = 0.25 * this->vehicle_length - this->vehicle_rear_hang;
        this->f2x = 0.75 * this->vehicle_length - this->vehicle_rear_hang;
    }
};

extern struct Vehicle_geometrics_ vehicle_geometrics_;
//vehicle kinematics settings
struct Vehicle_kinematics_ {
    double vehicle_v_max, vehicle_v_min, vehicle_a_max, vehicle_a_min, vehicle_jerk_max, vehicle_jerk_min;
    double vehicle_phi_max, vehicle_phi_min, vehicle_omega_max, vehicle_omega_min;
    double min_turning_radius;
    Vehicle_kinematics_() {
        this->vehicle_v_max = 2.5;
        this->vehicle_v_min = -2.5; // upper and lower bounds of v(t) (m/s)
        this->vehicle_a_max = 0.5;
        this->vehicle_a_min = -0.5; // uzpper and lower bounds of a(t) (m/s^2)
        this->vehicle_jerk_max = 0.5;
        this->vehicle_jerk_min = -0.5; // upper and lower bounds of jerk(t) (m/s^3) 
        this->vehicle_phi_max = 0.7;
        this->vehicle_phi_min = -0.7; // upper and lower bounds of phi(t) (rad)
        this->vehicle_omega_max = 0.5;
        this->vehicle_omega_min = -0.5; // upper and lower bounds of omega(t) (rad/s)

        this->min_turning_radius = vehicle_geometrics_.vehicle_wheelbase / tan(this->vehicle_phi_max);
    }
};

//scenario settings
struct Planning_scale_ {
    double xmin, xmax, ymin, ymax, x_scale, y_scale;
    Planning_scale_() {
        this->xmin = 0;
        this->xmax = 60;
        this->ymin = 0;
        this->ymax = 60;
        this->x_scale = this->xmax - this->xmin;
        this->y_scale = this->ymax - this->ymin;
    }
};


//vehicle initial and terminal states settings
struct Vehicle_TPBV_ {
    double x0, y0, theta0, v0, phi0, a, omega0, xtf, ytf, thetatf, vtf, phitf, omegatf;
    Vehicle_TPBV_() {
        this->x0 = 4;
        this->y0 = 4;
        this->theta0 = 0;
        this->v0 = 0;
        this->phi0 = 0;
        this->a = 0;
        this->omega0 = 0;

        this->xtf = 54;
        this->ytf = 54;
        this->thetatf = 0;
        this->vtf = 0;
        this->phitf = 0;
        this->omegatf = 0;
    }
};

extern struct Vehicle_kinematics_ vehicle_kinematics_;
extern struct Vehicle_TPBV_ vehicle_TPBV_;

extern int num_nodes_s;
extern double margin_obs_;
extern int Nobs;
extern cv::Mat costmap_;
vector<math::Vec2d> CreateVehiclePolygon(double x, double y, double theta);
vector<math::Vec2d> CreateVehiclePolygon(math::Vec2d p1, math::Vec2d p2, double theta);
double triArea(math::Vec2d p1, math::Vec2d p2, math::Vec2d p3);//  the area of triangle formed by the three points
bool checkObj_point(math::Vec2d p, vector<math::Vec2d> obj);// check point wheather in obstacle (based on area )
bool PtInPolygon(math::Vec2d p, vector<math::Vec2d>& ptPolygon);// check point wheather in Polygon (based on ray method )
bool checkObj_linev(math::Vec2d p1, math::Vec2d p2, vector<math::Vec2d> obj);
bool checkObj_linev(math::Vec2d p1, math::Vec2d p2);
bool PtInPolygon(double x, double y, vector<math::Vec2d>& ptPolygon);

double rand01();
vector < vector<math::Vec2d>> GenerateStaticObstacles_unstructured();
