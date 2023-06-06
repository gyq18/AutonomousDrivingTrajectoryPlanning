#include "CL_RRTStar.h"
#include <iostream>
#include <cmath>
#include <algorithm>
using namespace std;

double L_max = 3 * (vehicle_kinematics_.vehicle_v_max < 1.34) + 2.24 * vehicle_kinematics_.vehicle_v_max * (vehicle_kinematics_.vehicle_v_max >= 1.34 && vehicle_kinematics_.vehicle_v_max < 5.36) + 12 * (vehicle_kinematics_.vehicle_v_max >= 5.36);

CL_RRTStar::CL_RRTStar(State start,
               State goal,
               vector < vector<math::Vec2d>> &obstacles_,
               double sim_timestep,
               double expand_dis,
               double goal_sample_rate,
               double connect_circle_dist,
               double loc_threshold,
               double resolution_x,
               double resolution_y):
               _start(start),
               _goal(goal),
               _sim_timestep(sim_timestep),
               _expand_dis(expand_dis),
               _goal_sample_rate(goal_sample_rate),
               _connect_circle_dist(connect_circle_dist),
               _loc_threshold(loc_threshold),
               _resolution_x(resolution_x),
               _resolution_y(resolution_y)
               {
    //cout << "call CL_RRT" << endl;
    this->_obstacle_list = obstacles_;
    
    this->_grid_num_x = ceil(planning_scale_.x_scale / resolution_x);
    this->_grid_num_y = ceil(planning_scale_.y_scale / resolution_y);
    this->_costmap = new bool*[_grid_num_x];
    for (int i = 0; i < _grid_num_x; i++){
        this->_costmap[i] = new bool[_grid_num_y]();
    }
    this->_traj_buffer.reserve(256);
    this->_v_profile.reserve(256);
    this->_near_node_idxs.reserve(256);
    this->traj.reserve(256);
}

CL_RRTStar::~CL_RRTStar(){
    //cout << "call ~CL_RRTStar" << endl;
    for (int i = 0; i < _grid_num_x; i++){
        delete[] this->_costmap[i];
        this->_costmap[i] = nullptr;
    }
    delete[] this->_costmap;
    this->_costmap = nullptr;
}

void CL_RRTStar::planning(){
    this->CreateDilatedCostmap();
    double cmd[2] = {this->_start.x, this->_start.y};
    this->_node_list.push_back(Node(this->_start, cmd));
    int i;
    int nearest_node_idx, node_new_idx, goal_node_idx;
    
    while(1){
        this->get_random_sample();
        nearest_node_idx = this->find_nearest_node();
        node_new_idx = this->create_new_node(nearest_node_idx);
        if (node_new_idx == -1){
            continue;
        }
        this->rewire(node_new_idx);
        if(this->is_goal(node_new_idx)){
            break;
        }
    }

    //cout << "goal_node_idx" << goal_node_idx << endl;
    this->traj = this->get_traj(node_new_idx);
}

int CL_RRTStar::get_look_ahead_point_idx(const State &q){
        int idx = 0;
        double mindis = 10000;

        double Lf = 3 * (q.v < 1.34) + 2.24 * q.v * (q.v >= 1.34 && q.v < 5.36) + 12 * (q.v >= 5.36);
        for(int i = 0; i < REFERENCE_COURSE_LEN; i++){
            this->_dis_to_c[i] = hypot(q.x - this->_c_x[i], q.y - this->_c_y[i]);
            if (this->_dis_to_c[i] < mindis){
                mindis = this->_dis_to_c[i];
                idx = i;
            }
        }
        while(idx < REFERENCE_COURSE_LEN and this->_dis_to_c[idx] < Lf){
            idx += 1;
        }
        return min(idx, REFERENCE_COURSE_LEN);
}

void CL_RRTStar::get_random_sample(){
    if (((rand() % 10000) * 0.0001) < this->_goal_sample_rate){
        this->_ramdom_sample[0] = this->_goal.x;
        this->_ramdom_sample[1] = this->_goal.y;
        
    }
    else{
        double x = planning_scale_.xmin + planning_scale_.x_scale * (rand() % 100) * 0.01;
        double y = planning_scale_.ymin + planning_scale_.y_scale * (rand() % 100) * 0.01;
        this->_ramdom_sample[0] = x;
        this->_ramdom_sample[1] = y;
    }
}

int CL_RRTStar::create_new_node(int node_parent_idx){
    double dx = this->_ramdom_sample[0] - this->_node_list[node_parent_idx].cmd[0];
    double dy = this->_ramdom_sample[1] - this->_node_list[node_parent_idx].cmd[1];
    double norm = hypot(dx, dy);
    if(norm == 0){
        return -1;
    }
    double dis_to_goal = hypot(_node_list[node_parent_idx].q.x - _goal.x, _node_list[node_parent_idx].q.y - _goal.y);
    double _expend_dis = min(this->_expand_dis, dis_to_goal);
        

    double new_x = this->_node_list[node_parent_idx].cmd[0] + dx * _expand_dis / norm;
    double new_y = this->_node_list[node_parent_idx].cmd[1] + dy * _expand_dis / norm;

    new_x = min(new_x, planning_scale_.xmax);
    new_x = max(new_x, planning_scale_.xmin);
    new_y = min(new_y, planning_scale_.ymax);
    new_y = max(new_y, planning_scale_.ymin);
    
    double new_cmd[2];
    new_cmd[0] = new_x;
    new_cmd[1] = new_y;

    Node node_new(State(), new_cmd, this->_node_list[node_parent_idx].cost + this->_expand_dis);
    
    if(hypot(new_x - _goal.x, new_y - _goal.y) <= 3){
        node_new.terminate = true;
    }

    if (this->propagate(this->_node_list[node_parent_idx], node_new)){
        node_new.parent_idx = node_parent_idx;
        int end = this->_traj_buffer.size() - 1;
        //update state of node_new
        node_new.q = this->_traj_buffer[end];
        node_new.cost = this->_node_list[node_parent_idx].cost + _path_length;
        this->_node_list.push_back(node_new);
        return this->_node_list.size() - 1;
    }
    else
        return -1;
}

int CL_RRTStar::find_nearest_node(){
    int idx = 0;
    double mindis = 10000;
    double dis_temp;
    for(int i = 0; i < this->_node_list.size(); i++){
        if(!this->_node_list[i].reachable){
            continue;
        }
        dis_temp = dubin_dis(_node_list[i], _ramdom_sample);
        if(dis_temp < mindis){
            idx = i;
            mindis = dis_temp;
        }
    }
    return idx;
}

void CL_RRTStar::rewire(int node_new_idx){
    /**
     * @brief rewire the tree
     * 
     */
    this->find_near_nodes(node_new_idx);
    bool is_reparent = false;
    // find a new parent for node_new
    this->_dis.clear();
    for(int i = 0; i < _near_node_idxs.size(); i++){
        int &idx = this->_near_node_idxs[i];
        this->_dis.push_back(dubin_dis(_node_list[idx], _node_list[node_new_idx].cmd));
        if(this->_node_list[idx].reachable && this->_dis[i] + this->_node_list[idx].cost < this->_node_list[node_new_idx].cost){
            // check feasibility
            if (this->propagate(this->_node_list[idx], this->_node_list[node_new_idx])){
                this->_node_list[node_new_idx].parent_idx = idx;
                this->_node_list[node_new_idx].cost = _path_length + this->_node_list[idx].cost;
                int end = this->_traj_buffer.size() - 1;
                //update state of node_new
                this->_node_list[node_new_idx].q = this->_traj_buffer[end];
            }
        }
    }
    
    // rewire other nodes
    for(int i = 0; i < _near_node_idxs.size(); i++){
        int &idx = this->_near_node_idxs[i];
        if(this->_dis[i] + this->_node_list[node_new_idx].cost < this->_node_list[idx].cost){
            if(this->propagate(this->_node_list[node_new_idx], this->_node_list[idx])){
                this->_node_list[idx].parent_idx = node_new_idx;
                this->_node_list[idx].cost = _path_length + this->_node_list[node_new_idx].cost;
                
                int end = this->_traj_buffer.size() - 1;
                //update state
                this->_node_list[idx].q = this->_traj_buffer[end];
                is_reparent = true;
            }
        }
    }
    
    // update the tree
    if (is_reparent){
        this->update_tree(node_new_idx);
    }
}

bool CL_RRTStar::is_goal(int node_idx){
    /**
     * @brief decide where a node is near to goal enough
     * 
     */
    if (hypot(this->_goal.x - this->_node_list[node_idx].q.x, this->_goal.y - this->_node_list[node_idx].q.y) <= this->_loc_threshold){
        return (this->_node_list[node_idx].reachable && this->_node_list[node_idx].terminate);
    }
    return false;

}

bool CL_RRTStar::propagate(Node node_from, Node node_to){
    /**
     * @brief Propagate form node_from to node_to, using the control law describe in KuwataGNC08
     *        Stores the trajectory in _traj_buffer
     * @return true: Propagation succeed 
     *         false: Propagation failed
     */
    _path_length = 0;
    double t_min = 0.5;
    // form the controller input(draw a line)
    double step_x = (node_to.cmd[0] - node_from.cmd[0]) / (REFERENCE_COURSE_LEN - 1);
    double step_y = (node_to.cmd[1] - node_from.cmd[1]) / (REFERENCE_COURSE_LEN - 1);
    for(int i = 0; i < REFERENCE_COURSE_LEN; i++){
        this->_c_x[i] = node_from.cmd[0] + step_x * i;
        this->_c_y[i] = node_from.cmd[1] + step_y * i;
    }
    
    // get the initial look-ahead point
    int idx = this->get_look_ahead_point_idx(node_from.q);
    // generate v_profile
    // params for calculating v_coast
    double a_accel = fabs(vehicle_kinematics_.vehicle_a_max);
    double a_decel = fabs(vehicle_kinematics_.vehicle_a_min);
    double alpha_2 = -0.0252;
    double alpha_1 = 1.2344;
    double alpha_0 = -0.5347;
    // calculate v_coast

    double L_1 = 3 * (node_from.q.v < 1.34) + 2.24 * node_from.q.v * (node_from.q.v >= 1.34 && node_from.q.v < 5.36) + 12 * (node_from.q.v >= 5.36);
    double L_min;
    if(node_to.terminate)
        L_min = 3;
    else
        L_min = L_max;
    double v_coast;
    double D = hypot(this->_c_x[REFERENCE_COURSE_LEN - 1] - this->_c_x[idx], this->_c_y[REFERENCE_COURSE_LEN - 1] - this->_c_x[idx]);
    
    if (node_to.terminate){
        // vehicle stops at the end
        if (D > (vehicle_kinematics_.vehicle_v_max * vehicle_kinematics_.vehicle_v_max - node_from.q.v * node_from.q.v) 
          / (2 * vehicle_kinematics_.vehicle_a_max) + vehicle_kinematics_.vehicle_v_max * t_min 
          + vehicle_kinematics_.vehicle_v_max * vehicle_kinematics_.vehicle_v_max / (2 * a_decel) + alpha_2 * vehicle_kinematics_.vehicle_v_max * vehicle_kinematics_.vehicle_v_max + alpha_1 * vehicle_kinematics_.vehicle_v_max + alpha_0){
            v_coast = vehicle_kinematics_.vehicle_v_max;
        }
        else{
            double A = 1 / (2*a_accel) + 1 / (2*a_decel) + alpha_2;
            double B = t_min + alpha_1;
            double C = alpha_0 - D - node_from.q.v * node_from.q.v / (2 * a_accel);
            v_coast = (-1 * B + sqrt(B * B - 4 * A * C)) / (2 * A);
        }
        //cout << "v_coast:  " << v_coast << endl;
        this->_v_profile.clear();
        double v_t = node_from.q.v;
        this->_v_profile.push_back(v_t);
        int state = 0; // 0:ramp up; 1:coasting; 2:ramp down
        while(true){
            if(state == 0){
                v_t = min(v_coast, v_t + a_accel * this->_sim_timestep);
                if (v_t == v_coast)
                    state = 1;
            }
            else if(state == 1){
                v_t = v_coast;
                t_min -= this->_sim_timestep;
                if (t_min <= 0)
                    state = 2;
            }
            else if (state == 2){
                v_t = max(0.0, v_t - a_decel * this->_sim_timestep);
                if (v_t == 0){
                    this->_v_profile.push_back(v_t);
                    break;
                }
            }
            this->_v_profile.push_back(v_t);
        }
    }
    else{
        //vehicle does not stop at the end
        v_coast = vehicle_kinematics_.vehicle_v_max;
        this->_v_profile.clear();
        double v_t = node_from.q.v;
        this->_v_profile.push_back(v_t);
        int state = 0; // 0:ramp up; 1:coasting; 
        while(D >= 0){
            if(state == 0){
                v_t = min(v_coast, v_t + a_accel * _sim_timestep);
                D -= v_t * _sim_timestep;
                if (v_t == v_coast)
                    state = 1;
            }
            else if (state == 1){
                v_t = v_coast;
                D -= v_t * _sim_timestep;
            }
            _v_profile.push_back(v_t);
        }
    }
    
    
    State q_t(node_from.q);
    
    this->_traj_buffer.clear();
    _traj_buffer.push_back(q_t);
    idx = this->get_look_ahead_point_idx(q_t);
    // steering control
    double Lf = 3;
    double eta = q_t.yaw - atan2((this->_c_y[idx] - q_t.y), (this->_c_x[idx] - q_t.x));
    eta = pi_to_pi(eta);
    int move_dir;
    double delta;
    if(fabs(eta) < M_PI / 2){
        move_dir = 1; // forward
        delta = -1 * atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(eta) / Lf, 1.0);
    }
    else{
        move_dir = -1; // backward
        delta = -1 * atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(M_PI - eta) / Lf, 1.0);
    }
    
    // start simulation
    for (int i = 0; i < this->_v_profile.size(); i++){
        double &v = this->_v_profile[i];
        if (v == 0){
            continue;
        }
        idx = this->get_look_ahead_point_idx(q_t);
        // steering control
        Lf = 3 * (v < 1.34) + 2.24 * v * (v >= 1.34 && v < 5.36) + 12 * (v >= 5.36);
        eta = q_t.yaw - atan2((this->_c_y[idx] - q_t.y), (this->_c_x[idx] - q_t.x));
        eta = pi_to_pi(eta);
        //cout << "eta :  " << eta << endl;    
    
        if(fabs(eta) < M_PI / 2){
            delta = -atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(eta) / Lf, 1.0);
        }
        else{
            delta = -atan2(2.0 * vehicle_geometrics_.vehicle_length * sin(M_PI - eta) / Lf, 1.0);
        }
        if (delta > vehicle_kinematics_.vehicle_phi_max){
            delta = vehicle_kinematics_.vehicle_phi_max;
        }
            
        if (delta < vehicle_kinematics_.vehicle_phi_min){
            delta = vehicle_kinematics_.vehicle_phi_min;
        }
        //cout << "delta :  " << delta << endl;
    
        // update state
        q_t.x += v * _sim_timestep * cos(q_t.yaw) * move_dir;
        q_t.y += v * _sim_timestep * sin(q_t.yaw) * move_dir;
        q_t.v =  (v * move_dir);
        q_t.yaw += q_t.v / vehicle_geometrics_.vehicle_length * tan(delta);
        //cout << "yaw :  " << q_t.yaw << endl;
        q_t.yaw = pi_to_pi(q_t.yaw);
        _path_length += v * _sim_timestep;
        // check collision
        if(!this->check_collision(q_t)){
            return false;
        }
            
        this->_traj_buffer.push_back(q_t);
    }
    return true;
}

bool CL_RRTStar::check_collision(State q){
    int id_x = ConvertToIndex(q.x, 0);
    int id_y = ConvertToIndex(q.y, 1);
    return this->_costmap[id_x][id_y] == 0;
}

void CL_RRTStar::find_near_nodes(int node_new_idx){
    /**
     * @brief find near nodes, store the indexs in this->_near_node_idx
     * @return number of near nodes
     * 
     */
    this->_near_node_idxs.clear();
    double dis;
    for(int i = 0; i < this->_node_list.size(); i++){
        dis = dubin_dis(_node_list[i], _node_list[node_new_idx].cmd);
        if (dis < this->_connect_circle_dist){
            this->_near_node_idxs.push_back(i);
        }
    }
    return;
}

int CL_RRTStar::find_goal_node_idx(){
    /**
     * @brief find the index of node that is goal or closest to goal
     * 
     */
    double min_dis = 100000;
    int min_idx;
    double dis;
    for(int i = 0; i < this->_node_list.size(); i++){

        if (this->_node_list[i].reachable){
            dis = hypot(_node_list[i].q.x - _goal.x, _node_list[i].q.y - _goal.y);
            if(dis <= this->_loc_threshold){
                return i;
            }
            if(dis < min_dis){
                min_dis = dis;
                min_idx = i;
            }  
        }
    }        
    return min_idx;
}

vector<State> CL_RRTStar::get_traj(int goal_node_idx){
    vector<int> path;
    path.reserve(64);
    path.push_back(goal_node_idx);

    int parent = this->_node_list[goal_node_idx].parent_idx;
    while(parent != -1){
        path.push_back(parent);
        parent = this->_node_list[parent].parent_idx;
    }
    reverse(path.begin(), path.end());
    this->traj.clear();
    this->traj.push_back(_start);
    for(int i = 0; i < path.size() - 1; i++){
        this->propagate(_node_list[path[i]], _node_list[path[i + 1]]);
        this->traj.insert(this->traj.end(), _traj_buffer.begin() + 1, _traj_buffer.end());
    }        
    return traj;
}

void CL_RRTStar::update_tree(int parent_idx){
    /**
     * @brief update the tree after rewire()
     * 
     */
    for (int i = 0; i < _node_list.size(); i++){
        if (_node_list[i].parent_idx == parent_idx){
            if(_node_list[parent_idx].reachable){
                if(propagate(_node_list[parent_idx], _node_list[i])){
                    _node_list[i].cost = _path_length + _node_list[parent_idx].cost;
                    _node_list[i].q = _traj_buffer[_traj_buffer.size() -1];
                    _node_list[i].reachable = true;
                }
                else{
                    // denote node as unreachable
                    _node_list[i].reachable = false;
                }
            }
            else{
                _node_list[i].reachable = false;
            }
            update_tree(i);
        }
    }
}

double CL_RRTStar::pi_to_pi(double rad){
    while(rad > M_PI){
        rad -= 2 * M_PI;
    }
    while(rad < -1 * M_PI){
        rad += 2 * M_PI;
    }
    return rad;
}

void CL_RRTStar::CreateDilatedCostmap(){
    /**
    Generate Dilated Costmap
     **/
    double xmin = planning_scale_.xmin;
    double ymin = planning_scale_.ymin;
    

    double length_unit = 0.5 * (_resolution_x + _resolution_y);
    int dilate_radius = ceil(vehicle_geometrics_.radius / length_unit);
    
    for (int i = 0; i < Nobs; i++){ 
        vector<math::Vec2d> obstacles = _obstacle_list[i];
        double x_lb, x_ub, y_lb, y_ub;
        x_lb = obstacles[0].x();
        x_ub = obstacles[0].x();
        y_lb = obstacles[0].y();
        y_ub = obstacles[0].y();
        for(int i = 1; i < obstacles.size(); i++){
            x_lb = obstacles[i].x() < x_lb ? obstacles[i].x() : x_lb;
            x_ub = obstacles[i].x() > x_ub ? obstacles[i].x() : x_ub; 
            y_lb = obstacles[i].y() < y_lb ? obstacles[i].y() : y_lb;
            y_ub = obstacles[i].y() > y_ub ? obstacles[i].y() : y_ub; 
        }
        int Nmin_x = ConvertToIndex(x_lb, 0);
        int Nmax_x = ConvertToIndex(x_ub, 0);
        int Nmin_y = ConvertToIndex(y_lb, 1);
        int Nmax_y = ConvertToIndex(y_ub, 1);
        double cur_x, cur_y;
        for (int j = Nmin_x; j <= Nmax_x; j++){
            for (int k = Nmin_y; k <= Nmax_y; k++){
                if (this->_costmap[j][k] == 1){
                    continue;
                }
                cur_x = xmin + j * _resolution_x;
                cur_y = ymin + k * _resolution_y;
                if (PtInPolygon(cur_x, cur_y, obstacles)){
                    this->_costmap[j][k] = 1;
                    // dilate
                    for (int xx = max(0, j - dilate_radius); xx < min(j + dilate_radius, _grid_num_x); xx++){
                        for (int yy = max(0, k - dilate_radius); yy < min(k + dilate_radius, _grid_num_y); yy++){
                            if(this->_costmap[xx][yy] == 1){
                                continue;
                            }
                            if (floor(hypot(xx - j, yy - k)) < dilate_radius){
                                this->_costmap[xx][yy] = 1;
                            }
                        }
                    }
                }
            }
        }
    }
/*     for(int i = 0; i < _grid_num_x; i++){
        for(int j = 0; j < _grid_num_y; j++){
            cout << this->_costmap[i][j] << "  ";
        }
        cout << endl;
    } */
                        
}

int CL_RRTStar::ConvertToIndex(double x, int flag){
    if(flag == 0){
        //convert x
        int idx = ceil((x - planning_scale_.xmin) / _resolution_x);
        idx = min(idx, _grid_num_x - 1);
        idx = max(0, idx);
        return idx;
    }
    else{
        //convert y
        int idx = ceil((x - planning_scale_.ymin) / _resolution_y);
        idx = min(idx, _grid_num_y - 1);
        idx = max(0, idx);
        return idx;
    }
}

vector<double> CL_RRTStar::get_traj_x(){
    vector<double> x;
    x.reserve(this->traj.size());
    for(int i = 0; i < traj.size(); i++){
        x.push_back(traj[i].x);
    }
    return x;
}

vector<double> CL_RRTStar::get_traj_y(){
    vector<double> y;
    y.reserve(this->traj.size());
    for(int i = 0; i < traj.size(); i++){
        y.push_back(traj[i].y);
    }
    return y;

}

vector<double> CL_RRTStar::get_traj_theta(){
    vector<double> theta;
    theta.reserve(this->traj.size());
    for(int i = 0; i < traj.size(); i++){
        theta.push_back(traj[i].yaw);
    }
    return theta;
}

double CL_RRTStar::get_path_length(){
    double length = 0;
    for(int i = 0; i < this->traj.size() - 1; i++){
        length += hypot(this->traj[i + 1].x - this->traj[i].x, this->traj[i + 1].y - this->traj[i].y);
    }
    return length;
}

double CL_RRTStar::dubin_dis(Node node_from, double cmd[2]){
    double x = node_from.q.x;
    double y = node_from.q.y;
    double theta = node_from.q.yaw;
    double rho = vehicle_kinematics_.min_turning_radius;
    //now transform s.t. node is at (0,0,0) - RTMP 1109
    double sxx = cmd[0] - x;
    double syy = cmd[1] - y;
    double sx_rel = sxx * cos(theta) - syy * sin(-theta);
    double sy_rel = abs(sxx * sin(-theta) + syy * cos(theta));
    double dis;
    if  (hypot(sx_rel - 0, sy_rel - rho) < rho){ //relative point is in Dp  - RTMP 1109
        double df = hypot(sx_rel, (sy_rel + rho));
        double dc = hypot(sx_rel, sy_rel - rho);
        double theta_i = atan2(sx_rel, rho - sy_rel);
        double theta_c = (theta_i + 2 * M_PI);
        while(theta_c > 2 * M_PI){
            theta_c -= 2 * M_PI;
        }
        while(theta_c < 0){
            theta_c += 2 * M_PI;
        }
        double phi = acos((25*rho * rho - df * df)/(4 * rho * rho));
        double alpha = 2 * M_PI - phi;
        dis = rho*(alpha + asin(dc * sin(theta_c) / df) + asin((rho * sin(phi)) / df));
    }
    else{
        double dc = hypot(sx_rel, sy_rel - rho);
        double theta_i = atan2(sx_rel, rho - sy_rel);
        double theta_c = (theta_i + 2 * M_PI);
        while(theta_c > 2 * M_PI){
            theta_c -= 2 * M_PI;
        }
        while(theta_c < 0){
            theta_c += 2 * M_PI;
        }
        dis = sqrt(dc * dc - rho * rho) + rho * (theta_c - acos(rho/dc));
    }
    return dis;
}

    