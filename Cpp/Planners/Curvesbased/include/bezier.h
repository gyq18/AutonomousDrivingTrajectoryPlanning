#pragma once
#ifndef B_TRAJ
#define B_TRAJ

#include <Eigen/Eigen>
#include "vec2d.h"
#include "mathstruct.h"
#include <stdio.h>
#include <mosek.h>
#include "TransformP2T.h"
extern vector < vector<math::Vec2d>> obstacles_;
extern struct Vehicle_TPBV_ vehicle_TPBV_;
extern struct Vehicle_kinematics_ vehicle_kinematics_;
extern int Nobs;
using namespace std;
using namespace Eigen;
namespace Bezier
{
    /**
     *  bounding_box
     *  P1--------P2
     *   |        |
     *   |        |
     *   |        |
     *  P3--------P4
    **/
    struct bounding_box
    {
        math::Vec2d center;
        math::Vec2d P1;
        math::Vec2d P2;
        math::Vec2d P3;
        math::Vec2d P4;
        bounding_box() {};
        ~bounding_box() {};
        bounding_box(const math::Vec2d& pt) {
            center = pt;
            P1 = math::Vec2d(pt.x() - 0.01, pt.y() + 0.01);
            P2 = math::Vec2d(pt.x() + 0.01, pt.y() + 0.01);
            P3 = math::Vec2d(pt.x() - 0.01, pt.y() - 0.01);
            P4 = math::Vec2d(pt.x() + 0.01, pt.y() - 0.01);
        }
    };

    class bezier
    {
    private:
        MatrixXd pos;
        MatrixXd vel;
        MatrixXd acc;
        double maxVel;
        double minVel;
        double maxAcc;
        double minAcc;
        double maxJerk;
        double minJerk;
        vector<math::Vec2d> path;

        bounding_box box_last;
        vector<bounding_box> box_list;
        vector<bounding_box> vis_box_list;
        int max_inflate_iter = 1000;
        int traj_order = 6;
        int minimize_order = 4;
        double margin = 0.5;//change this to avoid collision(max = vehicle_geometrics_.radius), but this may lead to no solution

        MatrixXd MQM;

        vector<double> time_allocated;
        MatrixXd PolyCoeff;
        MatrixXd get_M();
        void set_MQM();
        void simplify_box();
        void Time_allocate();
        bool is_Overlap(const bounding_box& box_old, const bounding_box& box_now);
    public:
        bezier(vector<math::Vec2d> _path);
        ~bezier();
        void initialize();
        void inflate(const vector<math::Vec2d>& path);
        void inflate_box(bounding_box& box);
        bool is_in_box(const math::Vec2d& pt);
        int delete_box(const bounding_box& box_last, const bounding_box& box_now, const vector<math::Vec2d>& path);
        void update_vis_corridor();
        vector<bounding_box> get_box_list() {
            return vis_box_list;
        }
        double _C(int j);
        double factorial(const int& n);
        void get_Overlap_center(vector<math::Vec2d>& pt_list);
        int BezierPloyCoeffGeneration(double& obj);
        math::Vec2d getPosFromBezier(double t_now, int seg_now);
        struct Trajectory get_bezier_path(double path_length);
    };
}

#endif

