/*
 * @Author: lishengyong 
 * @Date: 2022-05-15 16:13:56
 * @LastEditors: lishengyong 
 * @LastEditTime: 2022-05-15 16:44:09
 * @FilePath: /cpp code/PlanSpeedForDynamicScenarios.cpp

 */


#include "PlanSpeedForDynamicScenarios.h"
#include "statelatticecpp/mathstruct.h"
#include "global_structs.h"
#include "GenerateDynamicObstacles_unstructured.h"
#include "Eigen/Dense"
#include "statelatticecpp/Visualize.h"
#include <ilcplex/ilocplex.h>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>
using namespace std;


PlanningResult PlanSpeedForDynamicScenarios(std::vector<double> x, std::vector<double> y, std::vector<double> theta)
{
    vector<int> t0;
    vector<int> s0;
    std::tie(t0, s0) = SearchVelocityInStGraph(x, y, theta);

    vector<double> t;
    vector<double> s;
    std::tie(t, s) = OptimizeVelocityInStGraph(t0, s0);

    return TransformPathToTrajectoryForDynamicScenarios(x, y, theta, t, s);

}

pair<vector<int>, vector<int>> SearchVelocityInStGraph(vector<double> x, vector<double> y, vector<double> theta)
{
    extern StGraphSearch st_graph_search_;
    extern Eigen::MatrixXd costmap_;
    costmap_ = Eigen::MatrixXd::Zero(st_graph_search_.num_nodes_t, st_graph_search_.num_nodes_s).eval();
    extern vector<vector<DObstacles>> dynamic_obs;

    int nobs = dynamic_obs.size();

    // generate costmap
    for (int i = 0; i < st_graph_search_.num_nodes_t; i++)
    {
        for (int j = 0; j < st_graph_search_.num_nodes_s; j++)
        {
            // calculate index
            int index = (int)(j * x.size() / st_graph_search_.num_nodes_s);
            auto current_x = x[index];
            auto current_y = y[index];
            auto current_theta = theta[index];

            for (int k = 0; k < nobs; k++)
            {
                if (IsVehicleCollidingWithMovingObstacle(current_x, current_y, current_theta, dynamic_obs[k][i]))
                {
                    costmap_(i, j) = 1;
                    continue;
                }
            }
        }
    }

    auto path = SearchStPathViaAStar();

    auto t = vector<int>();
    auto s = vector<int>();
    t.reserve(path.size());
    s.reserve(path.size());

    // extract path from vector<pair<>> to pair<vector<>, vector<>>
    for (int i =0; i< path.size(); i++)
    {
        auto ts = path[i];
        t.push_back(ts.first);
        s.push_back(ts.second);
    }

    return make_pair(t, s);

     
}

vector<pair<int, int>> SearchStPathViaAStar()
{
    extern StGraphSearch st_graph_search_;
    int num_t = st_graph_search_.num_nodes_t;
    int num_s = st_graph_search_.num_nodes_s;

    extern Eigen::MatrixXd costmap_;

    vector<vector<int>> expansion_pattern = {
        {0, 1},
        {0, -1},
        {1, 1},
        {1, -1},
        {1, 0}
    };
    vector<double> expansion_length = {
        1 + st_graph_search_.penalty_for_inf_velocity,
        1 + st_graph_search_.penalty_for_inf_velocity,
        1.414,
        1.414,
        1
    };

    // initialize first element in openlist
    Eigen::MatrixXd g_score = Eigen::MatrixXd::Constant(num_t, num_s, numeric_limits<double>::infinity()).eval();
    using ts_pair = pair<int, int>;
    ts_pair start_ts = make_pair(0, 0);
    ts_pair end_ts = make_pair(num_t-1, num_s - 1);
    g_score(start_ts.first, start_ts.second) = 0.0;
    double h_start = abs(end_ts.first - start_ts.first) + abs(end_ts.second - start_ts.second);

    priority_queue<pair<double, ts_pair>> open_list;
    set<ts_pair> close_list;

    // priority_queue is an max queue, use -f_score
    open_list.push(make_pair(-h_start, start_ts));
    map<ts_pair, ts_pair> path;

    while (!open_list.empty()) 
    {
        // pop the node with the min f_socre(max -fscore)
        double current_f;
        ts_pair current_ts;
        std::tie(current_f, current_ts) = open_list.top();
        open_list.pop();
        close_list.insert(current_ts);

        // if reach the end, break
        if (current_ts == end_ts)
        {
            break;
        }

        int move_num = expansion_length.size();
        for (int i = 0; i < move_num; i++)
        {
            auto move = expansion_pattern[i];
            auto cost = expansion_length[i];
            auto next_ts = make_pair(current_ts.first+move[0], current_ts.second+move[1]);

            // judge if next ts is valid
            if (!(next_ts.first >= 0 && next_ts.first < num_t && next_ts.second >= 0 && next_ts.second < num_s))
            {
                continue;
            }

            // if next_ts in close_list, continue
            if (close_list.find(next_ts) != close_list.end())
            {
                continue;
            }

            double temp_g_socre = g_score(current_ts.first, current_ts.second) + cost;

            if (temp_g_socre < g_score(next_ts.first, next_ts.second))
            {
                // if collide, continue
                if (costmap_(next_ts.first, next_ts.second) == 1)
                {
                    continue;
                }

                g_score(next_ts.first, next_ts.second) = temp_g_socre;
                double h_score = abs(end_ts.first - next_ts.first) + abs(end_ts.second - next_ts.second) * st_graph_search_.multiplier_H_for_A_star;
                double f_score = temp_g_socre + h_score;

                open_list.push(make_pair(-f_score, next_ts));
                path[next_ts] =  current_ts;

            }

        }
    }

    // if end_ts not in path, which means can not reach the end.
    // return empty vector.
    if (path.find(end_ts) == path.end())
    {
        return {};
    }

    // reconstruct the path
    vector<ts_pair> path_result;
    path_result.push_back(end_ts);
    auto current_ts = end_ts;
    while (path.find(current_ts) != path.end())
    {
        path_result.push_back(path[current_ts]);
        current_ts = path[current_ts];
    }
    reverse(path_result.begin(), path_result.end());

    return path_result;
}



int IsVehicleCollidingWithMovingObstacle(double x, double y, double theta, DObstacles obs)
{
    if (((obs.x - x).square() + (obs.y - y).square()).sqrt().minCoeff() > 10)
    {
        return 0;
    }

    auto Vcar = CreateVehiclePolygonFull(x, y, theta);

    // construct the obstacle polygon
    int obs_points = obs.x.size();
    vector<math::Vec2d> ptPolygon;
    for (int i = 0; i< obs_points; i++)
    {
        ptPolygon.push_back(math::Vec2d(obs.x[i], obs.y[i]));
    }

    // judge if point in vehicle lies inside the obstacles
    int point_num = Vcar.x.size();
    for (int i = 0; i< point_num; i++)
    {
        if (PtInPolygon(math::Vec2d(Vcar.x[i], Vcar.y[i]), ptPolygon))
        {
            return 1;
        }
    }


    return 0;

}

VehiclePolygonFull CreateVehiclePolygonFull(double x, double y, double theta)
{
    extern Vehicle_geometrics_ vehicle_geometrics_;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    VehiclePolygonFull Vcar;
    auto vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
    auto AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta -  vehicle_half_width * sin_theta;
    auto BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta +  vehicle_half_width * sin_theta;
    auto CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
    auto DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
    auto AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta +  vehicle_half_width * cos_theta;
    auto BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
    auto CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
    auto DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
    Vcar.x = {AX, BX, CX, DX, AX};
    Vcar.y = {AY, BY, CY, DY, AY};

    return Vcar;
}
    
pair<vector<double>, vector<double>> OptimizeVelocityInStGraph(vector<int> t0, vector<int> s0)
{
    using namespace Eigen;
    extern StGraphSearch st_graph_search_;
    extern Vehicle_kinematics_ vehicle_kinematics_;

    int ele_num = t0.size();
    double delta_t = st_graph_search_.resolution_t;
    
    auto t = t0;
    int nstep = s0.size();

    auto s0_vec = VectorXd(nstep);
    for (int i = 0; i < nstep; i++)
    {
        s0_vec(i) = s0[i];
    }

    // construct the matrix
    auto Q1 = MatrixXd::Identity(nstep, nstep);

    auto Vdiff = MatrixXd::Identity(nstep, nstep).eval();
    Vdiff.diagonal(1) -= VectorXd::Ones(nstep-1);
    Vdiff = (Vdiff * delta_t).eval(); 
    auto Q2 = (Vdiff(seq(0, last-1), all).transpose() * Q1(seq(1, last), seq(1, last)) * Vdiff(seq(0, last-1), all)).eval();
    
    auto Adiff = MatrixXd::Identity(nstep, nstep).eval();
    Adiff.diagonal(1) -= 2 * VectorXd::Ones(nstep-1);
    Adiff.diagonal(2) += VectorXd::Ones(nstep-2);
    Adiff = (Adiff * (delta_t * delta_t)).eval();
    auto Q3 = (Adiff(seq(0, last-2), all).transpose() * Adiff(seq(0, last-2), all)).eval();

    auto Jdiff = MatrixXd::Identity(nstep, nstep).eval();
    Jdiff.diagonal(1) -= 3 * VectorXd::Ones(nstep-1);
    Jdiff.diagonal(2) += 3* VectorXd::Ones(nstep-2);
    Jdiff.diagonal(3) -= VectorXd::Ones(nstep-3);
    Jdiff = (Jdiff * (delta_t * delta_t * delta_t)).eval();
    auto Q4 = (Jdiff(seq(0, last-2), all).transpose() * Jdiff(seq(0, last-2), all)).eval();

    vector<double> cref = {1, 0, 0, 0};
    vector<double> cabs = {0, 1, 1, 2};

    auto Qref = (Q1 * cref[0] + Q2 * cref[1] + Q3 * cref[2] + Q4 * cref[3]).eval();
    auto Qabs = (Q1 * cabs[0] + Q2 * cabs[1] + Q3 * cabs[2] + Q4 * cabs[3]).eval();

    auto Aeq = MatrixXd::Zero(2, nstep).eval();
    Aeq(0, 0) = 1;
    Aeq(1, nstep-1) = 1;
    MatrixXd beq = MatrixXd(2, 1);
    beq(0) = s0[0];
    beq(1) = s0[s0.size()-1];

    auto Aineq = MatrixXd((nstep-1)*2, nstep);
    Aineq.topRows(nstep-1) = Vdiff(seq(0, last-1), all);
    Aineq.bottomRows(nstep-1) = -Vdiff(seq(0, last-1), all);
    double vmax = vehicle_kinematics_.vehicle_v_max;
    double vmin = vehicle_kinematics_.vehicle_v_min;
    auto Bineq = MatrixXd((nstep-1)*2, 1);
    Bineq.topRows(nstep-1) = vmax * VectorXd::Ones(nstep-1);
    Bineq.bottomRows(nstep-1) = -vmin * VectorXd::Ones(nstep-1);



    auto H = (Qref + Qabs).eval();
    auto f = (-Qref * s0_vec).transpose().eval();

    auto lb = VectorXd::Zero(nstep).eval();
    auto ub = (s0_vec.maxCoeff() * VectorXd::Ones(nstep)).eval();

    // solve the qp
    auto s_res = solve_qp(H, f, Aineq, Bineq, Aeq, beq, lb, ub, s0_vec);

    vector<double> t_;
    for (int i = 0; i< t.size(); i++)
    {
        t_.push_back(t[i]);
    }

    return make_pair(t_, s_res);
}

vector<double> solve_qp(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd Aineq, Eigen::MatrixXd bineq, Eigen::MatrixXd Aeq,
    Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd x0)
{
    IloEnv env;
    vector<double> result;
    try {
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);

        int var_num = lb.size();

        // construct the problem
        // add variables and bounds 
        for (int i = 0; i < var_num; i++)
        {
            var.add(IloNumVar(env, lb(i), ub(i)));
        }
        

        // add eq bounds and ineq bounds
        int eq_row_num = Aeq.rows();
        for (int row = 0; row < eq_row_num; row++)
        {
            IloNumExprArg single_con = 0 * var[0];
            for (int j = 0; j < var_num; j++)
            {
                single_con = single_con + Aeq(row, j) * var[j];
            }
            con.add(single_con == beq(row));
        }

        int ineq_row_num = Aineq.rows();
        for (int row = 0; row < ineq_row_num; row ++)
        {
            IloNumExprArg single_con = 0 * var[0];
            for (int j = 0; j < var_num; j++)
            {
                single_con = single_con + Aineq(row, j) * var[j];
            }
            con.add(single_con <= bineq(row));
        }

        // add quadratic objective
        IloNumExprArg objective = 0 * var[0];
        for (int i = 0; i < var_num; i++)
        {
            for (int j = 0; j < var_num; j++)
            {
                objective = objective + var[i] * 0.5 * H(i, j) * var[j];            
            }
            objective = objective + var[i] * f(i);
        }
        model.add(IloMinimize(env, objective));
        model.add(con);


        IloCplex cplex(model);
        // disable output
        cplex.setOut(env.getNullStream());
        // cplex.setParam(IloCplex::Param::OptimalityTarget, 3);
        
        // solve the problem 
        if (!cplex.solve())
        {
            env.error() << "Failed to optimize QP" << endl;
            throw( -1);
        }

        IloNumArray vals(env);
        // env.out() << "Solution status = " << cplex.getStatus() << endl;
        // env.out() << "Solution value  = " << cplex.getObjValue() << endl;
        cplex.getValues(vals, var);
        // env.out() << "Values        = " << vals << endl;
        // cplex.getSlacks(vals, con);
        // env.out() << "Slacks        = " << vals << endl;
        // cplex.getDuals(vals, con);
        // env.out() << "Duals         = " << vals << endl;
        // cplex.getReducedCosts(vals, var);
        // env.out() << "Reduced Costs = " << vals << endl;

        // extract the result
        for (int i = 0; i< var_num; i++)
        {
            result.push_back(vals[i]);
        }

    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
       cerr << "Unknown exception caught" << endl;
    }

   env.end();

    return result;
}




PlanningResult TransformPathToTrajectoryForDynamicScenarios(vector<double> x, vector<double> y, vector<double> theta, vector<double>t, vector<double> s)
{
    extern StGraphSearch st_graph_search_;
    extern Vehicle_kinematics_ vehicle_kinematics_;
    extern Vehicle_geometrics_ vehicle_geometrics_;

    double delta_t = st_graph_search_.resolution_t;

    // remove the redundant ts where delta_t = 0
    if (t.size() > st_graph_search_.num_nodes_t)
    {
        vector<double> tt = {t[t.size()-1]};
        vector<double> ss = {s[t.size()-1]};

        for (int i = t.size()-2; i>=0; i--)
        {
            if (t[i+1] != t[i])
            {
                tt.push_back(t[i]);
                ss.push_back(s[i]);
            }
        }
        reverse(tt.begin(), tt.end());
        reverse(ss.begin(), ss.end());
        tt[0] = 0;
        ss[0] = 0;
        t = tt; 
        s = ss;
    }

    assert(t.size() == st_graph_search_.num_nodes_t);

    // sample more points to make trajectory smooth
    std::tie(x, y, theta) = ResamplePathWithEqualDistance( x, y, theta );

    auto ss = Eigen::ArrayXd::Zero(x.size()).eval();
    for(int i = 1 ; i< x.size(); i++)
    {
        ss[i] = ss[i-1] + hypot(x[i]-x[i-1], y[i]-y[i-1]);
    }


    ss = ss / ss.maxCoeff() * (*max_element(s.begin(), s.end()));

    PlanningResult trajectory;
    
    for (int i = 0; i< s.size(); i++)
    {
        Eigen::ArrayXd err = (ss.array() - s[i]).abs().eval();
        int ind_min = min_element(err.begin(), err.end()) - err.begin();

        trajectory.x.push_back(x[ind_min]);
        trajectory.y.push_back(y[ind_min]);
        trajectory.theta.push_back(theta[ind_min]);

    }

    int Nfe = trajectory.x.size();
    auto vdr = Eigen::ArrayXd::Zero(Nfe).eval();

    for (int i = 1; i< Nfe; i++)
    {
        double addition = (x[i+1] - x[i]) * cos(theta[i]) + (y[i+1] - y[i]) * sin(theta[i]);
        if (addition > 0)
        {
            vdr[i] = addition;
        } 
        else{
            vdr[i] = -addition;
        }
    }

    auto v = vector<double>(Nfe, 0);
    auto a = vector<double>(Nfe, 0);
    auto dt = delta_t;

    for (int i = 1; i< Nfe; i++)
    {
        v[i] = vdr[i] * hypot((x[i]-x[i-1])/dt, (y[i]-y[i-1])/dt);

    }

    for (int i = 1; i< Nfe; i++)
    {
        a[i] = (v[i] - v[i-1]) / dt;
    }

    auto phi = vector<double>(Nfe, 0);
    auto omega = vector<double>(Nfe, 0);
    double phi_max = vehicle_kinematics_.vehicle_phi_max;
    double omega_max = vehicle_kinematics_.vehicle_omega_max;

    for (int i = 1; i< Nfe-1; i++)
    {
        phi[i] = atan((theta[i+1] - theta[i])*vehicle_geometrics_.vehicle_wheelbase / (dt * v[i]));
        if (phi[i] > phi_max)
        {
            phi[i] = phi_max;
        }
        else if (phi[i] < -phi_max)
        {
            phi[i] = -phi_max;
        }
    }

    for (int i = 1; i< Nfe-1; i++)
    {
        omega[i] = (phi[i+1] - phi[i]) / dt;
        if (omega[i] > omega_max)
        {
            omega[i] = omega_max;
        }
        else if (omega[i] < -omega_max)
        {
            omega[i] = -omega_max;
        }
    }

    trajectory.v = v;
    trajectory.a = a;
    trajectory.phi = phi;
    trajectory.omega = omega;

    

    return trajectory;

}

std::tuple<vector<double>, vector<double>, vector<double>> ResamplePathWithEqualDistance(vector<double>x, vector<double> y,
 vector<double> theta)
{
    // make theta continuous
    for(int i = 1; i < theta.size(); i++)
    {
        while((theta[i] - theta[i-1]) > M_PI)
        {
            theta[i] -= 2 * M_PI;
        }
        while((theta[i] - theta[i-1]) < -M_PI)
        {
            theta[i] += 2 * M_PI;
        }
    }

    vector<double> x_extended;
    vector<double> y_extended;
    vector<double> theta_extended;

    // sample more points of x, y, theta
    for (int i = 0; i < theta.size()-1; i++)
    {
        double distance = hypot(x[i+1] - x[i], y[i+1] - y[i]);
        long LARGE_NUM = (long)round(distance * 100);

        auto temp_x = Eigen::ArrayXd::LinSpaced(LARGE_NUM, x[i], x[i+1]).eval();
        x_extended.insert(x_extended.end(), temp_x.begin(), temp_x.end());

        auto temp_y = Eigen::ArrayXd::LinSpaced(LARGE_NUM, y[i], y[i+1]).eval();
        y_extended.insert(y_extended.end(), temp_y.begin(), temp_y.end());

        auto temp_theta = Eigen::ArrayXd::LinSpaced(LARGE_NUM, theta[i], theta[i+1]).eval();
        theta_extended.insert(theta_extended.end(), temp_theta.begin(), temp_theta.end());

    }

    x_extended.push_back(x[x.size()-1]);
    y_extended.push_back(y[y.size()-1]);
    theta_extended.push_back(theta[theta.size()-1]);

    // use linspace to get the index 
    auto index = Eigen::ArrayXd::LinSpaced(1000, 0, x_extended.size()-1).cast<int>().eval();

    auto x_res = vector<double>(index.size());
    auto y_res = vector<double>(index.size());
    auto theta_res = vector<double>(index.size());

    transform(index.begin(), index.end(), x_res.begin(), [&x_extended](int x){
        return x_extended[x];
    });
    transform(index.begin(), index.end(), y_res.begin(), [&y_extended](int x){
        return y_extended[x];
    });
    transform(index.begin(), index.end(), theta_res.begin(), [&theta_extended](int x){
        return theta_extended[x];
    });


    return make_tuple(x, y, theta);
}

void VisualizeDynamicResultsForDynamicScenarios(PlanningResult& trajectory, string filename,
    int frame_speed)
{
    using namespace cv;

    auto x = trajectory.x;
    auto y = trajectory.y;
    auto theta = trajectory.theta;

    extern Vehicle_TPBV_ vehicle_TPBV_;
    extern vector<vector<math::Vec2d> > obstacles_;
    extern int Nobs;
    extern vector<vector<DObstacles>> dynamic_obs;
    extern Planning_scale_ planning_scale_;
    

    int number_of_frame = x.size();
    auto x0 = vehicle_TPBV_.x0;
    auto y0 = vehicle_TPBV_.y0;
    auto theta0 = vehicle_TPBV_.theta0;
    auto xtf = vehicle_TPBV_.xtf;
    auto ytf = vehicle_TPBV_.ytf;
    auto thetatf = vehicle_TPBV_.thetatf;

    int nstep = trajectory.x.size();
    double xmax = *max_element(trajectory.x.begin(), trajectory.x.end());//最大值
    double xmin = *min_element(trajectory.x.begin(), trajectory.x.end());//最小值
    double ymax = *max_element(trajectory.y.begin(), trajectory.y.end());//最大值
    double ymin = *min_element(trajectory.y.begin(), trajectory.y.end());//最小值
    cout << "max_x: " << xmax << endl;
    cout << "min_x: " << xmin << endl;
    cout << "max_y: " << ymax << endl;
    cout << "min_y: " << ymin << endl;

    // here I use drawframe_dynamic to render the image
    cv_axis2d axis(xmin - 5, xmax + 5, ymin - 5, ymax + 5);
    Mat canvas = Mat::ones(Size(512, 512 * (axis.y_h - axis.y_l) / (axis.x_h - axis.x_l)), CV_8UC3);

    auto video_writer = VideoWriter(filename, VideoWriter::fourcc('m', 'p', '4', 'v'), frame_speed,  
        canvas.size());

    
    // draw frames
    for (int i = 0;i < nstep;++i) {        
        canvas.setTo(255);
        drawframe_dynamic(trajectory, i, axis, canvas);
        video_writer.write(canvas);
    }




}

// draw a frame in the i'th step
void drawframe_dynamic(PlanningResult trajectory,int i, cv_axis2d axis, cv::Mat canvas) {

    extern vector<vector<DObstacles>> dynamic_obs;
    extern vector<vector<math::Vec2d> > obstacles_;

    // draw static obs
    int nstep = trajectory.x.size();
    if (Nobs > 0) {
        for (int j = 0; j < Nobs;++j) {
            vector<Point> pts = axis.plist_in_canvas(obstacles_[j], canvas);
            fillPoly(canvas, pts, Scalar(0.7451 * 255, 0.7451 * 255, 0.7451 * 255));
        }
    }

    // draw dynamic obs
    if (dynamic_obs.size() > 0)
    {
        for (int j = 0; j < dynamic_obs.size(); j++)
        {
            vector<math::Vec2d> temp;
            for (int k = 0; k < 4; k++)
            {
                temp.push_back(math::Vec2d(dynamic_obs[j][i].x[k], dynamic_obs[j][i].y[k]));
            }
            auto pts = axis.plist_in_canvas(temp, canvas);
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