#pragma once
#include <string>
#include <vector>
#include "mathstruct.h"

#define REFERENCE_COURSE_LEN 50

using namespace std;

extern Vehicle_geometrics_ vehicle_geometrics_;
extern Vehicle_kinematics_ vehicle_kinematics_;
extern Vehicle_TPBV_ vehicle_TPBV_;
extern Planning_scale_ planning_scale_;
extern vector < vector<math::Vec2d>> obstacles_;

typedef struct State{
    double x;
    double y;
    double yaw;
    double v;
    State(double x = 0,
          double y = 0,
          double v = 0,
          double yaw = 0):
          x(x), y(y), v(v), yaw(yaw){};
    State(const State &p){
        this->x = p.x;
        this->y = p.y;
        this->yaw = p.yaw;
        this->v = p.v;
    };
    State &operator=(const State &p){
        this->x = p.x;
        this->y = p.y;
        this->yaw = p.yaw;
        this->v = p.v;
        return *this;
    };
}State;

typedef struct Node
{
    State q;
    double cmd[2];
    int parent_idx;
    double cost;
    bool reachable;
    bool terminate;
    Node(State q = State(), double cmd[2] = {0}, double cost = 0): parent_idx(-1), cost(cost), reachable(true), terminate(false){
        this->q = q;
        memcpy(this->cmd, cmd, 2 * sizeof(cmd[0]));
    };
}Node;


class CL_RRTStar{
    private:
        // attributes
        State _start; // start state;
        State _goal; // goal state;
        double _sim_timestep; // timestep for simulation
        double _expand_dis;
        double _goal_sample_rate;
        double _connect_circle_dist;
        double _loc_threshold;
        bool _search_until_max_iter;
        vector<Node> _node_list;
        bool **_costmap;
        double _resolution_x;
        double _resolution_y;
        int _grid_num_x;
        int _grid_num_y;

        // buffer variables
        double _ramdom_sample[2]; // random sample buffer,[x, y];
        double _c_x[REFERENCE_COURSE_LEN]; // reference course x;
        double _c_y[REFERENCE_COURSE_LEN]; // reference course y;
        double _dis_to_c[REFERENCE_COURSE_LEN]; // distance to reference course;
        vector<double> _dis; // distance to certain node of each node;
        vector<int> _near_node_idxs; // indexs of near_node 
        vector<State> _traj_buffer;
        vector<double> _v_profile;
        vector < vector<math::Vec2d>> _obstacle_list;
        double _path_length;
        // methods
        int get_look_ahead_point_idx(const State& q);
        void get_random_sample();
        int create_new_node(int node_parent_idx);
        int find_nearest_node();
        void rewire(int node_new_idx);
        bool is_goal(int node_idx);
        bool propagate(Node node_from, Node node_to);
        bool check_collision(State q);
        void find_near_nodes(int node_new_idx);
        int find_goal_node_idx();
        vector<State> get_traj(int goal_node_idx);
        void update_tree(int parent_idx);
        inline double pi_to_pi(double rad);
        void CreateDilatedCostmap();
        int ConvertToIndex(double x, int flag);
        double dubin_dis(Node node_from, double cmd[2]);
    public:        
        vector<State> traj;
        vector<double> get_traj_x();
        vector<double> get_traj_y();
        vector<double> get_traj_theta();
        double get_path_length();
        CL_RRTStar(State start,
               State goal,
               vector < vector<math::Vec2d>> &obstacles_,
               double sim_timestep=0.1,
               double expand_dis=30.0,
               double goal_sample_rate=0,
               double connect_circle_dist=50.0,
               double loc_threshold = 0.5,
               double resolution_x = 0.5,
               double resolution_y = 0.5);
        ~CL_RRTStar();
        void planning();
};