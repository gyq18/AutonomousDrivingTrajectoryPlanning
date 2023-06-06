#include "bezier.h"
#include <time.h>
using namespace Bezier;

bezier::bezier(vector<math::Vec2d> _path) {
    path = _path;
    initialize();
};

bezier::~bezier() {};

void bezier::initialize() {
    // p,v,a
    pos = MatrixXd::Zero(2, 2);
    vel = MatrixXd::Zero(2, 2);
    acc = MatrixXd::Zero(2, 2);
    pos.row(0) = Vector2d(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    pos.row(1) = Vector2d(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    vel.row(0) = Vector2d(vehicle_TPBV_.v0, vehicle_TPBV_.v0);
    vel.row(1) = Vector2d(vehicle_TPBV_.vtf, vehicle_TPBV_.vtf);
    acc.row(0) = Vector2d(vehicle_TPBV_.a, vehicle_TPBV_.a);
    acc.row(1) = Vector2d(vehicle_TPBV_.a, vehicle_TPBV_.a);
    maxVel = vehicle_kinematics_.vehicle_v_max;
    minVel = vehicle_kinematics_.vehicle_v_min;
    maxAcc = vehicle_kinematics_.vehicle_a_max;
    minAcc = vehicle_kinematics_.vehicle_a_min;
    maxJerk = vehicle_kinematics_.vehicle_jerk_max;
    minJerk = vehicle_kinematics_.vehicle_jerk_min;
    //MQM Matrix
    set_MQM();
    //get corridor
    inflate(path);
    //time allocate
    Time_allocate();
}
void bezier::update_vis_corridor() {
    vis_box_list = box_list;
}

void bezier::inflate(const vector<math::Vec2d>& path) {
    box_list.clear();
    math::Vec2d start(pos(0, 0), pos(0, 1));
    box_last = bounding_box(start);
    int n = path.size();
    for (int i = 0; i < n; i++) {
        if (is_in_box(path[i])) {
            continue;
        }
        bounding_box box_now(path[i]);

        inflate_box(box_now);

        int flag = delete_box(box_last, box_now, path);
        if (flag == 1) {
            //do nothing  return 1
        }
        else if (flag == 2) {
            if (!box_list.empty())
                box_list.pop_back(); //return 2

            while (!box_list.empty())//
            {
                int size = box_list.size();
                box_last = box_list[size - 1];
                int flag2 = delete_box(box_last, box_now, path);
                if (flag2 != 2)
                    break;
                box_list.pop_back();
            }

            box_list.push_back(box_now);
            box_last = box_now;
        }
        else {
            box_list.push_back(box_now);
            box_last = box_now;
        }

    }
    if (box_list.size() > 1)
        simplify_box();
    update_vis_corridor();
}

void bezier::simplify_box() {
    auto temp = box_list;
    box_list.clear();
    int n = temp.size();
    int idx_old = 0;
    box_list.push_back(temp[idx_old]);
    for (int i = 1; i < n; ++i) {
        if (!is_Overlap(temp[idx_old], temp[i])) {
            // cout<<i<<endl;
            box_list.push_back(temp[i - 1]);
            idx_old = i - 1;
            if (is_Overlap(temp[i - 1], temp[n - 1])) {//one shot
                break;
            }
        }
    }
    box_list.push_back(temp[n - 1]);
}

bool bezier::is_Overlap(const bounding_box& box_old, const bounding_box& box_now) {
    if (box_now.P3.x() >= (box_old.P2.x() + 1) || box_now.P3.y() >= (box_old.P2.y() + 1) ||//
        box_old.P3.x() >= (box_now.P2.x() + 1) || box_old.P3.y() >= (box_now.P2.y() + 1)) {
        return false;
    }
    return true;
}

double bezier::factorial(const int& n) {
    double res = 1.0;
    for (int i = n; i >= 1; --i) {
        res *= i;
    }
    return res;
}

void bezier::set_MQM() {
    int box_nums = box_list.size();
    MatrixXd Q = MatrixXd::Zero(traj_order + 1, traj_order + 1);
    MatrixXd M = MatrixXd::Zero(traj_order + 1, traj_order + 1);
    MQM = MatrixXd::Zero(traj_order + 1, traj_order + 1);
    for (int i = 3; i <= traj_order; ++i) {
        for (int j = 3; j <= traj_order; ++j) {
            Q(i, j) = (double)(i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2)) / (double)(i + j - 5);
        }
    }
    M = get_M();
    MQM = M.transpose() * Q * M;
}

void bezier::Time_allocate() {
    time_allocated = vector<double>(box_list.size(), 200);
    vector<math::Vec2d> pt_list;
    math::Vec2d start(pos(0, 0),pos(0,1));
    pt_list.push_back(start);
    get_Overlap_center(pt_list);
    math::Vec2d goal(pos(1, 0), pos(1, 1));
    pt_list.push_back(goal);
    for (int i = 0; i < pt_list.size() - 1; ++i) {
        double dis = pt_list[i + 1].DistanceTo(pt_list[i]);
        double t = dis / (maxVel * 0.1);
        time_allocated[i] = t;
    }
}

void bezier::get_Overlap_center(vector<math::Vec2d>& pt_list) {
    for (int i = 0; i < box_list.size() - 1; ++i) {
        double x_low = max(box_list[i].P1.x(), box_list[i + 1].P1.x());
        double x_up = min(box_list[i].P2.x(), box_list[i + 1].P2.x());
        double y_low = max(box_list[i].P4.y(), box_list[i + 1].P4.y());
        double y_up = min(box_list[i].P2.y(), box_list[i + 1].P2.y());
        math::Vec2d pt((x_low + x_up) / 2, (y_low + y_up) / 2);
        pt_list.push_back(pt);
    }
}

MatrixXd bezier::get_M() {
    MatrixXd M = MatrixXd::Zero(traj_order + 1, traj_order + 1);
    switch (traj_order)
    {
    case 0:
    {
        M << 1;
        break;
    }
    case 1:
    {
        M << -1, 0,
            -1, 1;
        break;
    }
    case 2:
    {
        M << -1, 0, 0,
            -2, 2, 0,
            1, -2, 1;
        break;
    }
    case 3:
    {
        M << -1, 0, 0, 0,
            -3, 3, 0, 0,
            3, -6, 3, 0,
            -1, 3, -3, 1;
        break;
    }
    case 4:
    {
        M << 1, 0, 0, 0, 0,
            -4, 4, 0, 0, 0,
            6, -12, 6, 0, 0,
            -4, 12, -12, 4, 0,
            1, -4, 6, -4, 1;
        break;
    }
    case 5:
    {
        M << 1, 0, 0, 0, 0, 0,
            -5, 5, 0, 0, 0, 0,
            10, -20, 10, 0, 0, 0,
            -10, 30, -30, 10, 0, 0,
            5, -20, 30, -20, 5, 0,
            -1, 5, -10, 10, -5, 1;
        break;
    }
    case 6:
    {
        M << 1, 0, 0, 0, 0, 0, 0,
            -6, 6, 0, 0, 0, 0, 0,
            15, -30, 15, 0, 0, 0, 0,
            -20, 60, -60, 20, 0, 0, 0,
            15, -60, 90, -60, 15, 0, 0,
            -6, 30, -60, 60, -30, 6, 0,
            1, -6, 15, -20, 15, -6, 1;
        break;
    }
    case 7:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0,
            -7, 7, 0, 0, 0, 0, 0, 0,
            21, 42, 21, 0, 0, 0, 0, 0,
            -35, 105, -105, 35, 0, 0, 0, 0,
            35, -140, 210, -140, 35, 0, 0, 0,
            -21, 105, -210, 210, -105, 21, 0, 0,
            7, -42, 105, -140, 105, -42, 7, 0,
            -1, 7, -21, 35, -35, 21, -7, 1;
        break;
    }
    case 8:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0, 0,
            -8, 8, 0, 0, 0, 0, 0, 0, 0,
            28, -56, 28, 0, 0, 0, 0, 0, 0,
            -56, 168, -168, 56, 0, 0, 0, 0, 0,
            70, -280, 420, -280, 70, 0, 0, 0, 0,
            -56, 280, -560, 560, -280, 56, 0, 0, 0,
            28, -168, 420, -560, 420, -168, 28, 0, 0,
            -8, 56, -168, 280, -280, 168, -56, 8, 0,
            1, -8, 28, -56, 70, -56, 28, -8, 1;
        break;
    }
    case 9:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -9, 9, 0, 0, 0, 0, 0, 0, 0, 0,
            36, -72, 36, 0, 0, 0, 0, 0, 0, 0,
            -84, 252, -252, 84, 0, 0, 0, 0, 0, 0,
            126, -504, 756, -504, 126, 0, 0, 0, 0, 0,
            -126, 630, -1260, 1260, -630, 126, 0, 0, 0, 0,
            84, -504, 1260, -1680, 1260, -504, 84, 0, 0, 0,
            -36, 252, -756, 1260, -1260, 756, -252, 36, 0, 0,
            9, -72, 252, -504, 630, -504, 252, -72, 9, 0,
            -1, 9, -36, 84, -126, 126, -84, 36, -9, 1;
        break;
    }
    case 10:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            45, -90, 45, 0, 0, 0, 0, 0, 0, 0, 0,
            -120, 360, -360, 120, 0, 0, 0, 0, 0, 0, 0,
            210, -840, 1260, -840, 210, 0, 0, 0, 0, 0, 0,
            -252, 1260, -2520, 2520, -1260, 252, 0, 0, 0, 0, 0,
            210, -1260, 3150, -4200, 3150, -1260, 210, 0, 0, 0, 0,
            -120, 840, -2520, 4200, -4200, 2520, -840, 120, 0, 0, 0,
            45, -360, 1260, -2520, 3150, -2520, 1260, -360, 45, 0, 0,
            -10, 90, -360, 840, -1260, 1260, -840, 360, -90, 10, 0,
            1, -10, 45, -120, 210, -252, 210, -120, 45, -10, 1;
        break;
    }
    case 11:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -11, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            55, -110, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -165, 495, -495, 165, 0, 0, 0, 0, 0, 0, 0, 0,
            330, -1320, 1980, -1320, 330, 0, 0, 0, 0, 0, 0, 0,
            -462, 2310, -4620, 4620, -2310, 462, 0, 0, 0, 0, 0, 0,
            462, -2772, 6930, -9240, 6930, -2772, 462, 0, 0, 0, 0, 0,
            -330, 2310, -6930, 11550, -11550, 6930, -2310, 330, 0, 0, 0, 0,
            165, -1320, 4620, -9240, 11550, -9240, 4620, -1320, 165, 0, 0, 0,
            -55, 495, -1980, 4620, -6930, 6930, -4620, 1980, -495, 55, 0, 0,
            11, -110, 495, -1320, 2310, -2772, 2310, -1320, 495, -110, 11, 0,
            -1, 11, -55, 165, -330, 462, -462, 330, -165, 55, -11, 1;
        break;
    }
    case 12:
    {
        M << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            66, -132, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -220, 660, -660, 220, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            495, -1980, 2970, -1980, 495, 0, 0, 0, 0, 0, 0, 0, 0,
            -792, 3960, -7920, 7920, -3960, 792, 0, 0, 0, 0, 0, 0, 0,
            924, -5544, 13860, -18480, 13860, -5544, 924, 0, 0, 0, 0, 0, 0,
            -792, 5544, -16632, 27720, -27720, 16632, -5544, 792, 0, 0, 0, 0, 0,
            495, -3960, 13860, -27720, 34650, -27720, 13860, -3960, 495, 0, 0, 0, 0,
            -220, 1980, -7920, 18480, -27720, 27720, -18480, 7920, -1980, 220, 0, 0, 0,
            66, -660, 2970, -7920, 13860, -16632, 13860, -7920, 2970, -660, 66, 0, 0,
            -12, 132, -660, 1980, -3960, 5544, -5544, 3960, -1980, 660, -132, 12, 0,
            1, -12, 66, -220, 495, -792, 924, -792, 495, -220, 66, -12, 1;
        break;
    }
    }
    return M;
}

int bezier::delete_box(const bounding_box& box_last, const bounding_box& box_now, const vector<math::Vec2d>& path) {
    int n = path.size();
    bool has_box_last = false, has_box_now = false;
    for (int i = n - 1; i >= 0; --i) {
        int Px = path[i].x(), Py = path[i].y();
        if (!has_box_last) {
            if (Px >= box_last.P1.x() && Px <= box_last.P2.x() && Py >= box_last.P4.y() && Py <= box_last.P2.y() &&
                (Px<box_now.P1.x() || Px>box_now.P2.x() || Py<box_now.P4.y() || Py>box_now.P2.y()))
            {
                has_box_last = true;//
            }
        }
        if (!has_box_now) {
            if (Px >= box_now.P1.x() && Px <= box_now.P2.x() && Py >= box_now.P4.y() && Py <= box_now.P2.y() &&
                (Px<box_last.P1.x() || Px>box_last.P2.x() || Py<box_last.P4.y() || Py>box_last.P2.y()))
            {
                has_box_now = true;//
            }
        }
        if (has_box_last && has_box_now) {
            return 0;
        }
    }
    if (has_box_last && !has_box_now)
        return 1;

    return 2;
}

bool bezier::is_in_box(const math::Vec2d& pt) {
    if (box_list.empty()) {
        return false;
    }
    if (pt.x() >= box_list.back().P1.x() && pt.x() <= box_list.back().P2.x() && pt.y() >= box_list.back().P4.y() && pt.y() <= box_list.back().P2.y())
        return true;
    return false;
}

void bezier::inflate_box(bounding_box& box) {
    bool has_P2_x = false, has_P2_y = false, has_P3_x = false, has_P3_y = false, stop = false;
    math::Vec2d temp_P2 = box.P2;
    math::Vec2d temp_P3 = box.P3;
    double step = 0.3;
    for (int j = 0; j < max_inflate_iter; ++j) {

        // cout<<i<<endl;
        // cout<<temp_P2(0)<<" "<<temp_P2(1)<<" "<<temp_P3(0)<<" "<<temp_P3(1)<<endl;
        if (!has_P2_x || !has_P2_y) {
            if (!has_P2_x) {

                temp_P2.set_y(temp_P2.y() + step);
                math::Vec2d temp_P1(temp_P3.x(), temp_P2.y());
                for (int i = 0; i < Nobs; i++) {
                    if (temp_P2.y() >= planning_scale_.ymax || checkObj_linev(temp_P1, temp_P2, obstacles_[i])) {
                        has_P2_x = true;
                        break;
                    }
                }

                if (has_P2_x) {
                    temp_P2.set_y(temp_P2.y() - step);
                }
            }

            if (!has_P2_y) {
                temp_P2.set_x(temp_P2.x() + step);
                math::Vec2d temp_P4(temp_P2.x(), temp_P3.y());
                for (int i = 0; i < Nobs; i++) {
                    if (temp_P2.x() >= planning_scale_.xmax || checkObj_linev(temp_P2, temp_P4, obstacles_[i])) {
                        has_P2_y = true;
                        break;
                    }
                }
                if (has_P2_y) {
                    temp_P2.set_x(temp_P2.x() - step);
                }
            }
        }
        if (!has_P3_x || !has_P3_y) {
            if (!has_P3_x) {
                temp_P3.set_y(temp_P3.y() - step);
                math::Vec2d temp_P4(temp_P2.x(), temp_P3.y());
                for (int i = 0; i < Nobs; i++) {
                    if (temp_P3.y() <= planning_scale_.ymin || checkObj_linev(temp_P3, temp_P4, obstacles_[i])) {
                        has_P3_x = true;
                        break;
                    }
                }

                if (has_P3_x) {
                    temp_P3.set_y(temp_P3.y() + step);
                }
            }
            if (!has_P3_y) {
                temp_P3.set_x(temp_P3.x() - step);
                math::Vec2d temp_P1(temp_P3.x(), temp_P2.y());
                // if (temp_P3.x() <= planning_scale_.xmin || checkObj_linev(temp_P3, temp_P1)) {
                //     has_P3_y = true;
                // }
                for (int i = 0; i < Nobs; i++) {
                    if (temp_P3.x() <= planning_scale_.xmin || checkObj_linev(temp_P3, temp_P1, obstacles_[i])) {
                        has_P3_y = true;
                        break;
                    }
                }

                if (has_P3_y) {
                    temp_P3.set_x(temp_P3.x() + step);
                }
            }
        }

        if (has_P2_x && has_P2_y && has_P3_x && has_P3_y)
            break;
    }

    box.P2.set_y(temp_P2.y());
    box.P1.set_y(temp_P2.y());

    box.P2.set_x(temp_P2.x());
    box.P4.set_x(temp_P2.x());

    box.P3.set_y(temp_P3.y());
    box.P4.set_y(temp_P3.y());

    box.P3.set_x(temp_P3.x());
    box.P1.set_x(temp_P3.x());
    // cout<<2<<endl;
}


double bezier::_C(int j) {
    return factorial(traj_order) / (factorial(j) * factorial(traj_order - j));
}

math::Vec2d bezier::getPosFromBezier(double t_now, int seg_now)
{
    int dim = 2;
    Vector2d ret = VectorXd::Zero(2);
    VectorXd ctrl_now = PolyCoeff.row(seg_now);
    int ctrl_num1D = PolyCoeff.cols() / dim;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (traj_order - j));
    math::Vec2d point(ret(0), ret(1));
    return point;
}

struct Trajectory bezier::get_bezier_path(double path_length) {
    vector<math::Vec2d> path;
    for (int i = 0; i < box_list.size(); i++) {
        for (double t = 0; t < time_allocated[i]; t += time_allocated[i] / 6) {
            double t_now = 1 - (time_allocated[i] - t) / time_allocated[i];
            math::Vec2d path_pt = time_allocated[i] * getPosFromBezier(t_now, i);
            path.push_back(path_pt);
        }
        // if(i!=box_list.size()-1){
        //     path.pop_back();
        // }
    }
    vector<double> x;
    vector<double> y;
    vector<double> theta;

    for (unsigned int i = 0; i < path.size() - 1; i++) {
        x.push_back(path[i].x());
        y.push_back(path[i].y());
        double angle = (path[i + 1] - path[i]).Angle();
        theta.push_back(angle);
    }
    x.push_back(path.back().x());
    y.push_back(path.back().y());
    theta.push_back(0);

    struct Trajectory traj = TransformPathToTrajectory(x, y, theta, path_length, 1);
    return traj;
}

int bezier::BezierPloyCoeffGeneration(double& obj) {
    double initScale = time_allocated.front();
    double lstScale = time_allocated.back();
    int segment_num = box_list.size();
    int dim = 2;
    int n_poly = traj_order + 1;
    int s1d1CtrlP_num = n_poly;
    int s1CtrlP_num = dim * s1d1CtrlP_num;

    int equ_con_s_num = dim * 3; // p, v, a in x, y axis at the start point
    int equ_con_e_num = dim * 3; // p, v, a in x, y axis at the end point
    int equ_con_continuity_num = dim * 4 * (segment_num - 1);
    int equ_con_num = equ_con_s_num + equ_con_e_num + equ_con_continuity_num; // p, v, a in x, y axis in each segment's joint position

    int vel_con_num = dim * traj_order * segment_num;
    int acc_con_num = dim * (traj_order - 1) * segment_num;
    int jer_con_num = dim * (traj_order - 2) * segment_num;

    int high_order_con_num = vel_con_num + acc_con_num + jer_con_num;

    int con_num = equ_con_num + high_order_con_num;
    int ctrlP_num = segment_num * s1CtrlP_num;

    double* x_var = new double[ctrlP_num];
    double primalobj;

    MSKrescodee  r;
    vector< pair<MSKboundkeye, pair<double, double> > > con_bdk;

    /***  Stack the bounding value for the linear inequality for the v constraints  ***/
    for (int i = 0; i < vel_con_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair(MSK_BK_RA, make_pair(minVel, +maxVel));
        con_bdk.push_back(cb_ie);
    }

    /***  Stack the bounding value for the linear inequality for the a constraints  ***/
    for (int i = 0; i < acc_con_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair(MSK_BK_RA, make_pair(minAcc, maxAcc));
        con_bdk.push_back(cb_ie);
    }

    /***  Stack the bounding value for the linear inequality for the j constraints  ***/
    for (int i = 0; i < jer_con_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair(MSK_BK_RA, make_pair(minJerk, maxJerk));
        con_bdk.push_back(cb_ie);
    }


    for (int i = 0; i < equ_con_num; i++) {
        double beq_i;
        //start p_x p_y v_x v_y a_x a_y
        if (i < dim)                    beq_i = pos(0, i);
        else if (i >= dim && i < dim*2) beq_i = vel(0, i - dim);
        else if (i >= dim*2 && i < dim*3) beq_i = acc(0, i - dim*2);
        //end
        else if (i >= dim*3 && i < dim*4) beq_i = pos(1, i - dim*3);
        else if (i >= dim*4 && i < dim*5) beq_i = vel(1, i - dim*4);
        else if (i >= dim*5 && i < dim*6) beq_i = acc(1, i - dim*5);
        //continuity
        else beq_i = 0.0;
        pair<MSKboundkeye, pair<double, double> > cb_eq = make_pair(MSK_BK_FX, make_pair(beq_i, beq_i)); // # cb_eq means: constriants boundary of equality constrain
        con_bdk.push_back(cb_eq);
    }
  
    /* ## define a container for control points' boundary and boundkey ## */
    /* ## dataType in one tuple is : boundary type, lower bound, upper bound ## */
    vector< pair<MSKboundkeye, pair<double, double> > > var_bdk;

    for (int k = 0; k < segment_num; k++)
    {
        Bezier::bounding_box cube_ = box_list[k];
        double scale_k = time_allocated[k];

        for (int i = 0; i < dim; i++)
        {
            for (int j = 0; j < n_poly; j++)
            {
                pair<MSKboundkeye, pair<double, double> > vb_x;

                double lo_bound, up_bound;
                if (i == 0) {
                    if (k > 0)
                    {
                        lo_bound = (cube_.P1.x() + margin) / scale_k;
                        up_bound = (cube_.P2.x() - margin) / scale_k;
                    }
                    else
                    {
                        lo_bound = (cube_.P1.x()) / scale_k;
                        up_bound = (cube_.P2.x()) / scale_k;
                    }
                }
                if (i == 1) {
                    if (k > 0)
                    {
                        lo_bound = (cube_.P3.y() + margin) / scale_k;
                        up_bound = (cube_.P2.y() - margin) / scale_k;
                    }
                    else
                    {
                        lo_bound = (cube_.P3.y()) / scale_k;
                        up_bound = (cube_.P2.y()) / scale_k;
                    }
                }

                vb_x = make_pair(MSK_BK_RA, make_pair(lo_bound, up_bound)); 

                var_bdk.push_back(vb_x);
            }
        }
    }
  
    MSKint32t  j, i;
    MSKenv_t   env;
    MSKtask_t  task;
    // Create the mosek environment. 
    r = MSK_makeenv(&env, NULL);

    // Create the optimization task. 
    r = MSK_maketask(env, con_num, ctrlP_num, &task);

    // Parameters used in the optimizer
    //######################################################################
    //MSK_putintparam (task, MSK_IPAR_OPTIMIZER , MSK_OPTIMIZER_INTPNT );
    MSK_putintparam(task, MSK_IPAR_NUM_THREADS, 1);
    MSK_putdouparam(task, MSK_DPAR_CHECK_CONVEXITY_REL_TOL, 1e-2);
    MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_DFEAS, 1e-4);
    MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_PFEAS, 1e-4);
    MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, 1e-4);
    //MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_REL_GAP, 5e-2 );
    //######################################################################

    //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr); 
    // Append empty constraints. 
    //The constraints will initially have no bounds. 
    if (r == MSK_RES_OK)
        r = MSK_appendcons(task, con_num);

    // Append optimizing variables. The variables will initially be fixed at zero (x=0). 
    if (r == MSK_RES_OK)
        r = MSK_appendvars(task, ctrlP_num);

    for (j = 0; j < ctrlP_num && r == MSK_RES_OK; ++j) {
        if (r == MSK_RES_OK)
            r = MSK_putvarbound(task,
                j,                            // Index of variable. 
                var_bdk[j].first,             // Bound key.
                var_bdk[j].second.first,      // Numerical value of lower bound.
                var_bdk[j].second.second);   // Numerical value of upper bound.      
    }

    // Set the bounds on constraints. 
    for (i = 0; i < con_num && r == MSK_RES_OK; i++) {
        r = MSK_putconbound(task,
            i,                            // Index of constraint. 
            con_bdk[i].first,             // Bound key.
            con_bdk[i].second.first,      // Numerical value of lower bound.
            con_bdk[i].second.second);   // Numerical value of upper bound. 
    }

    int row_idx = 0;
    // The velocity constraints
    for (int k = 0; k < segment_num; k++)
    {
        for (int i = 0; i <dim; i++)
        {  // for x, y loop
            for (int p = 0; p < traj_order; p++)
            {
                int nzi = 2;
                MSKint32t* asub = new MSKint32t[nzi];
                double* aval = new double[nzi];

                aval[0] = -1.0 * traj_order;
                aval[1] = 1.0 * traj_order;

                asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
                asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx++;
            }
        }
    }

    // The acceleration constraints
    for (int k = 0; k < segment_num; k++)
    {
        for (int i = 0; i < dim; i++)
        {
            for (int p = 0; p < traj_order - 1; p++)
            {
                int nzi = 3;
                MSKint32t* asub = new MSKint32t[nzi];
                double* aval = new double[nzi];

                aval[0] = 1.0 * traj_order * (traj_order - 1) / time_allocated[k];
                aval[1] = -2.0 * traj_order * (traj_order - 1) / time_allocated[k];
                aval[2] = 1.0 * traj_order * (traj_order - 1) / time_allocated[k];
                asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
                asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;
                asub[2] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 2;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx++;
            }
        }
    }

    for (int k = 0; k < segment_num; k++)
    {
        for (int i = 0; i < dim; i++)
        {
            for (int p = 0; p < traj_order - 2; p++)
            {
                int nzi = 4;
                MSKint32t* asub = new MSKint32t[nzi];
                double* aval = new double[nzi];

                aval[0] = -1.0 * traj_order * (traj_order - 1)*(traj_order-2) / time_allocated[k];
                aval[1] = 3.0 * traj_order * (traj_order - 1)*(traj_order-2) / time_allocated[k];
                aval[2] = -3.0 * traj_order * (traj_order - 1)*(traj_order-2) / time_allocated[k];
                aval[3] = 1.0 * traj_order * (traj_order - 1) * (traj_order - 2) / time_allocated[k];
                asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
                asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;
                asub[2] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 2;
                asub[3] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 3;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx++;
            }
        }
    }
    /*   Start position  */
    
    // position :
    for (int i = 0; i < dim; i++)
    {     
        int nzi = 1;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        aval[0] = 1.0 * initScale;
        asub[0] = i * s1d1CtrlP_num;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }
    // velocity :
    for (int i = 0; i < dim; i++)
    {        
        int nzi = 2;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        aval[0] = -1.0 * traj_order;
        aval[1] = 1.0 * traj_order;
        asub[0] = i * s1d1CtrlP_num;
        asub[1] = i * s1d1CtrlP_num + 1;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }
    // acceleration : 
    for (int i = 0; i < dim; i++)
    {   
        int nzi = 3;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        aval[0] = 1.0 * traj_order * (traj_order - 1) / initScale;
        aval[1] = -2.0 * traj_order * (traj_order - 1) / initScale;
        aval[2] = 1.0 * traj_order * (traj_order - 1) / initScale;
        asub[0] = i * s1d1CtrlP_num;
        asub[1] = i * s1d1CtrlP_num + 1;
        asub[2] = i * s1d1CtrlP_num + 2;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }
    

    /*   End position  */
    // position :
    for (int i = 0; i < dim; i++)
    {  // loop for x, y, z       
        int nzi = 1;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        asub[0] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num;
        aval[0] = 1.0 * lstScale;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }
    // velocity :
    for (int i = 0; i < dim; i++)
    {
        int nzi = 2;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        asub[0] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num - 1;
        asub[1] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num;
        aval[0] = -1.0;
        aval[1] = 1.0;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }
    // acceleration : 
    for (int i = 0; i < dim; i++)
    {
        int nzi = 3;
        MSKint32t* asub = new MSKint32t[nzi];
        double* aval = new double[nzi];
        asub[0] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num - 2;
        asub[1] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num - 1;
        asub[2] = ctrlP_num - 1 - (dim-1 - i) * s1d1CtrlP_num;
        aval[0] = 1.0 / lstScale;
        aval[1] = -2.0 / lstScale;
        aval[2] = 1.0 / lstScale;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
    }

    /*   joint points  */
    int sub_shift = 0;
    double val0, val1;
    for (int k = 0; k < (segment_num - 1); k++)
    {
        double scale_k = time_allocated[k];
        double scale_n = time_allocated[k + 1];
        // position :
        val0 = scale_k;
        val1 = scale_n;
        for (int i = 0; i < dim; i++)
        {
            int nzi = 2;
            MSKint32t* asub = new MSKint32t[nzi];
            double* aval = new double[nzi];

            // This segment's last control point
            aval[0] = 1.0 * val0;
            asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;

            // Next segment's first control point
            aval[1] = -1.0 * val1;
            asub[1] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }

        for (int i = 0; i < dim; i++)
        {
            int nzi = 4;
            MSKint32t* asub = new MSKint32t[nzi];
            double* aval = new double[nzi];

            // This segment's last velocity control point
            aval[0] = -1.0;
            aval[1] = 1.0;
            asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 2;
            asub[1] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;
            // Next segment's first velocity control point
            aval[2] = 1.0;
            aval[3] = -1.0;

            asub[2] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
            asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;

            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }
        // acceleration :
        val0 = 1.0 / scale_k;
        val1 = 1.0 / scale_n;
        for (int i = 0; i < dim; i++)
        {
            int nzi = 6;
            MSKint32t* asub = new MSKint32t[nzi];
            double* aval = new double[nzi];

            // This segment's last acceleration control point
            aval[0] = 1.0 * val0;
            aval[1] = -2.0 * val0;
            aval[2] = 1.0 * val0;
            asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 3;
            asub[1] = sub_shift + (i + 1) * s1d1CtrlP_num - 2;
            asub[2] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;
            // Next segment's first acceleration control point
            aval[3] = -1.0 * val1;
            aval[4] = 2.0 * val1;
            aval[5] = -1.0 * val1;
            asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
            asub[4] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
            asub[5] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 2;

            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }

        // jerk :
        val0 = 1.0 / pow(scale_k,2);
        val1 = 1.0 / pow(scale_n,2);
        for (int i = 0; i < dim; i++)
        {
            int nzi = 8;
            MSKint32t* asub = new MSKint32t[nzi];
            double* aval = new double[nzi];

            // This segment's last jerk control point
            aval[0] = -1.0 * val0;
            aval[1] = 3.0 * val0;
            aval[2] = -3.0 * val0;
            aval[3] = 1 * val0;
            asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 4;
            asub[1] = sub_shift + (i + 1) * s1d1CtrlP_num - 3;
            asub[2] = sub_shift + (i + 1) * s1d1CtrlP_num - 2;
            asub[3] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;
            // Next segment's first jerk control point
            aval[4] = 1.0 * val1;
            aval[5] = -3.0 * val1;
            aval[6] = 3.0 * val1;
            aval[7] = -1 * val1;
            asub[4] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
            asub[5] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
            asub[6] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 2;
            asub[7] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 3;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx++;
        }
        sub_shift += s1CtrlP_num;
    }


    //set Q
    int NUMQNZ = 0;
    for (int i = 0; i < segment_num; i++)
    {
        int NUMQ_blk = (traj_order + 1);
        NUMQNZ += dim * NUMQ_blk * (NUMQ_blk + 1) / 2;
    }

    MSKint32t* qsubi = new MSKint32t[NUMQNZ];
    MSKint32t* qsubj = new MSKint32t[NUMQNZ];
    double* qval = new double[NUMQNZ];

    sub_shift = 0;
    int idx = 0;
    for (int k = 0; k < segment_num; k++)
    {
        double scale_k = time_allocated[k];
        for (int p = 0; p < dim; p++)
            for (int i = 0; i < s1d1CtrlP_num; i++)
                for (int j = 0; j < s1d1CtrlP_num; j++)
                    if (i >= j)
                    {
                        qsubi[idx] = sub_shift + p * s1d1CtrlP_num + i;
                        qsubj[idx] = sub_shift + p * s1d1CtrlP_num + j;
                        
                        qval[idx] = MQM(i, j) / (double)pow(scale_k, 2 * minimize_order - 3);
                        idx++;
                    }

        sub_shift += s1CtrlP_num;
    }
 
    clock_t t_start, t_end;
    t_start = clock();

    if (r == MSK_RES_OK)
        r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);

    if (r == MSK_RES_OK)
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

    
    bool solve_ok = false;
    if (r == MSK_RES_OK)
    {
        
        MSKrescodee trmcode;
        r = MSK_optimizetrm(task, &trmcode);
        MSK_solutionsummary(task, MSK_STREAM_LOG);

        if (r == MSK_RES_OK)
        {
            MSKsolstae solsta;
            MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

            switch (solsta)
            {
            case MSK_SOL_STA_OPTIMAL:
                r = MSK_getxx(task,
                    MSK_SOL_ITR,
                    x_var);

                r = MSK_getprimalobj(
                    task,
                    MSK_SOL_ITR,
                    &primalobj);

                obj = primalobj;
                solve_ok = true;

                break;

            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER:
                printf("Primal or dual infeasibility certificate found.\n");
                break;

            case MSK_SOL_STA_UNKNOWN:
                printf("The status of the solution could not be determined.\n");
                break;
            default:
                printf("Other solution status.");
                break;
            }
        }
        else
        {
            printf("Error while optimizing.\n");
        }
    }

    if (r != MSK_RES_OK)
    {
        // In case of an error print error code and description. 
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];

        printf("An error occurred while optimizing.\n");
        MSK_getcodedesc(r,
            symname,
            desc);
        printf("Error %s - '%s'\n", symname, desc);
    }

    MSK_deletetask(&task);
    MSK_deleteenv(&env);
    t_end = clock();
    
    std::cout << "optimization time is " << t_end - t_start << std::endl;

    if (!solve_ok) {
        std::cout << "In solver, falied " << endl;
        return -1;
    }

    VectorXd d_var(ctrlP_num);
    for (int i = 0; i < ctrlP_num; i++)
        d_var(i) = x_var[i];

    PolyCoeff = MatrixXd::Zero(segment_num, dim * (traj_order + 1));

    int var_shift = 0;
    for (int i = 0; i < segment_num; i++)
    {
        for (int j = 0; j < dim * n_poly; j++)
            PolyCoeff(i, j) = d_var(j + var_shift);

        var_shift += dim * n_poly;
    }
    return 1;
}