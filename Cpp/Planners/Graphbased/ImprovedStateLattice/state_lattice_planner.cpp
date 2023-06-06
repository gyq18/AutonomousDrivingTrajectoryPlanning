#include "state_lattice_planner.h"
#include <fstream>
#include <iostream>
#include "mathstruct.h"
#include <queue>
#include <algorithm>
#include <chrono>

using namespace std;

// global variables
extern Vehicle_geometrics_ vehicle_geometrics_;
extern Vehicle_kinematics_ vehicle_kinematics_;
extern Hybrid_astar_ hybrid_astar_;
extern Vehicle_TPBV_ vehicle_TPBV_;
extern Planning_scale_ planning_scale_;
extern vector<vector<math::Vec2d> > obstacles_;


void LUT::load_LUT(string filename)
{
    ifstream f;
    stringstream buffer;

    f.open(filename);
    assert(f.is_open());
    buffer << f.rdbuf();
    string s = buffer.str();

    json result = json::parse(s.data());
    // load the edge_length
    this->edge_length = result["edge_length"].get<int>();
    // load the lut data.
    this->lut = vector<vector<double>>(8);
    for(int i = 0; i < this->direction_num; i++)
    {
        this->lut[i] = result[to_string(i)].get<vector<double>>();
    }
}

double LUT::look_up(int start_direction, Position to_pos, int to_direction) const 
{
    int index = to_pos.x() * this->edge_length * this->direction_num + to_pos.y() * this->direction_num
        + to_direction;
    return this->lut[start_direction][index];
}

int theta2direction(double theta)
{
    return (int)round(theta / M_PI_4) % 8;
}

double direction2theta(int direction)
{
    return direction * M_PI_4;
}

Position loc2pos(Vec2d loc, double scale)
{
    double xmin = planning_scale_.xmin;
    double ymin = planning_scale_.ymin;

    return Position((int)floor((loc.x() - xmin) / scale), (int)floor((loc.y() - ymin) / scale) );
}

Vec2d pos2loc(Position pos, double scale)
{
    double xmin = planning_scale_.xmin;
    double ymin = planning_scale_.ymin;
    return Vec2d(pos.x() * scale + xmin, pos.y() *scale + ymin);
}

Vec2d pos2loc(Vec2d pos, double scale)
{
    double xmin = planning_scale_.xmin;
    double ymin = planning_scale_.ymin;
    return Vec2d(pos.x() * scale + xmin, pos.y() *scale + ymin);
}

pair<vector<double>, vector<NodeIndex>> Astar(const StateLatticeGraph & graph, 
    Position start_pos, int start_direction, Position end_pos, int end_direction,
    std::function<double(const StateLatticeGraph &, Position, int, Position, int)> heuristic_func,
    std::function<bool(const StateLatticeGraph &, Position, int, Position, int)> pos_validate
)
{
    // define the path. Each element is the previous node index of current node index
    auto path = vector<NodeIndex>(graph.node_num, -1);
    const auto & nodes = graph.nodes;
    const auto & edges = graph.edges;

    NodeIndex start_index = graph.position2index(start_pos, start_direction);
    NodeIndex end_index = graph.position2index(end_pos, end_direction);

    auto close_index = vector<NodeIndex>();
    // open_list is optimized by priority_queue
    priority_queue<pair<double, NodeIndex>> open_list;

    // calculate g_score, h_score, f_score = g_score + h_score.
    auto g_score = vector<double>(graph.node_num, numeric_limits<double>::infinity());
    g_score[start_index] = 0.0;
    auto h_score = heuristic_func(graph, start_pos, start_direction, end_pos, end_direction);
    auto f_start = h_score + g_score[start_index];

    // warning, remember to use -fscore in priority_queue
    // beacuse the priority_queue always pop the max
    open_list.push(make_pair(-f_start, start_index));

    while (!open_list.empty())
    {
        // pop the node with the min f_score.
        auto temp_record = open_list.top();
        open_list.pop();
        NodeIndex current_index = temp_record.second;
        auto temp_result = graph.index2position(current_index);
        Position current_pos = temp_result.first;
        int current_direction = temp_result.second;
        // put it to close list
        close_index.push_back(current_index);

        if (current_index == end_index)
        {
            break;
        }

        const auto & current_edges = edges[current_index];
        for (auto iter = current_edges.begin(); iter < current_edges.end(); iter++)
        {
            NodeIndex adj_index = iter->index;
            double edge_cost = iter->cost;
            
            // if adj_index in close_index then continue
            if (find(close_index.begin(), close_index.end(), adj_index) != close_index.end())
            {
                continue;
            }
            auto temp_result = graph.index2position(adj_index);
            auto adj_pos = temp_result.first;
            auto adj_direction = temp_result.second;

            // check if the move from current to adj is valid.
            if (!pos_validate(graph, current_pos, current_direction, adj_pos, adj_direction))
            {
                // if not valid. Then the adj is not connect to the current.
                // continue
                continue;
            }
            
            double temp_g_score = g_score[current_index] + edge_cost;
            if (temp_g_score < g_score[adj_index])
            {
                path[adj_index] = current_index;
                g_score[adj_index] = temp_g_score;

                auto f_score = g_score[adj_index] + 
                    heuristic_func(graph, adj_pos, adj_direction, end_pos, end_direction);
                // warning, remember to use -fscore in priority_queue
                open_list.push(make_pair(-f_score, adj_index));
            }

        }
        
    }
    return make_pair(g_score, path);

}

bool check_move(const StateLatticeGraph & graph, Position from_pos, int from_direction, 
    Position to_pos, int to_direction, double loc2pos_scale)
{
    auto from_loc = pos2loc(from_pos, loc2pos_scale);
    auto to_loc = pos2loc(to_pos, loc2pos_scale);
    vector<math::Vec2d> V = CreateVehiclePolygon(to_loc.x(), to_loc.y(), direction2theta(to_direction));
    cout << V.size() << endl;

    auto iter = obstacles_.begin();
    for(; iter < obstacles_.end(); iter++)
    {   
        // we assume that the from_pos is valid.
        if (checkObj_point(to_loc, *iter))
        {
            return false;
        }
        if (checkObj_linev(V[0], V[1], *iter)) {
            return false;
        }
        if (checkObj_linev(V[1], V[2], *iter)) {
            return false;
        }
        if (checkObj_linev(V[2], V[3], *iter)) {
            return false;
        }
        if (checkObj_linev(V[3], V[0], *iter)) {
            return false;
        }
        // TODO: it seems I can check all obstacles in one time.
        if (checkObj_linev(from_loc, to_loc, *iter))
        {
            return false;
        }
    }
    return true;
}

double hlut(const LUT & lut, const StateLatticeGraph & graph, Position start_pos, int start_direction, Position end_pos, int end_direction)
{
    auto edge_length = lut.edge_length;
    auto delta_pos = end_pos - start_pos;
    
    // if the delta_position is bigger then the lut,
    // the lut can not give an precise result.
    // we use overflow_facotr to estimate the result.
    double overflow_factor = 1;

    int delta_x = delta_pos.x(); 
    int delta_y = delta_pos.y();

    if (delta_x < 0)
    {
        // reflect with t-axis
        delta_x = - delta_x;
        start_direction = ((4 - start_direction) + 8) % 8;
        end_direction = ((4 - end_direction) + 8) % 8;
    } 

    if (delta_y < 0)
    {
        // reflect with x-axis
        delta_y = - delta_y;
        start_direction = ((- start_direction) + 8) % 8;
        end_direction = ((- end_direction) + 8) % 8;
    }
    
    // if delta_x < edge_length && delta_y < endge_length 
    // then directly return
    if (delta_x < edge_length && delta_y < edge_length)
    {
        return lut.look_up(start_direction, Position(delta_x, delta_y), end_direction);
    }

    // use Euclidean distance to esitimate
    return std::hypot(delta_x, delta_y);
}

void path_interpolation(PlanningResult & result, const vector<pair<Position, int>>& path_result_list,
    const ActionSet & action_set, double loc2pos_scale)
{
    // add more points to make trajectory smooth
    auto iter = path_result_list.begin();
    vector<vector<double>> theta_full;
    for(; iter < path_result_list.end() - 1; iter++)
    {   
        auto start_pos = iter->first;
        auto start_direction = iter->second;
        auto end_pos = (iter+1)->first;
        auto end_direction = (iter+1)->second;
        auto points_result = action_set.sample_points(start_pos, start_direction, end_pos, end_direction);
        
        // convert position to absolute location
        auto temp_x_result = points_result.first.first;
        auto temp_y_result = points_result.first.second;
        auto x_len = temp_x_result.size();
        for (int i = 0; i < x_len; i++)
        {
            Vec2d temp_pos = Vec2d(temp_x_result[i], temp_y_result[i]);
            Vec2d loc_res = pos2loc(temp_pos, loc2pos_scale);
            result.x.push_back(loc_res.x());
            result.y.push_back(loc_res.y());
        }
        auto temp_theta_result = points_result.second;
        result.theta.insert(result.theta.end(), temp_theta_result.begin(), temp_theta_result.end());
    }
}


PlanningResult ImprovedStateLatticePlan(double loc2pos_scale, string lut_file, string insert_points_file)
{
    // process scaling. this will define how big is the graph.
    double graph_width = planning_scale_.x_scale();
    double graph_height = planning_scale_.y_scale();
    auto action_set = ActionSet();
    action_set.load_data(insert_points_file);
    auto graph = StateLatticeGraph((int)(graph_width / loc2pos_scale), (int)(graph_height / loc2pos_scale), action_set);

    Vec2d start_loc = Vec2d(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    int start_direction = theta2direction(vehicle_TPBV_.theta0);
    Vec2d end_loc = Vec2d(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    int end_direction = theta2direction(vehicle_TPBV_.thetatf);

    // convert loc to pos
    Position start_pos = loc2pos(start_loc, loc2pos_scale);
    Position end_pos = loc2pos(end_loc, loc2pos_scale);

    NodeIndex start_index = graph.position2index(start_pos, start_direction);
    NodeIndex end_index = graph.position2index(end_pos, end_direction);

    LUT lut;
    lut.load_LUT(lut_file);
    // cout << "load successfully" << endl;

    // In C++, lambda function with variables captured can not be 
    // cast to a function pointer.
    // I have to use std::function to pass the closures.
    std::function<double(const StateLatticeGraph &, Position, int, Position, int)> heuristic_func = 
    [&](const StateLatticeGraph & g, Position s_p, int s_d, Position e_p, int e_d) 
    {
        // static int call_times = 0;
        // call_times++;
        // cout << "call hlut: " << call_times << endl;
        return hlut(lut, g, s_p, s_d, e_p, e_d);
    };
    std::function<bool(const StateLatticeGraph &, Position, int, Position, int)> position_check = 
    [&](const StateLatticeGraph & g, Position s_p, int s_d, Position e_p, int e_d)
    {
        return check_move(g, s_p, s_d, e_p, e_d, loc2pos_scale);
    };

    auto start_time = chrono::high_resolution_clock::now();
    auto astar_result = Astar(graph, start_pos, start_direction, end_pos, end_direction, heuristic_func, position_check);
    auto end_time = chrono::high_resolution_clock::now();
    std::cout << "Astar time consumed: " << 
        chrono::duration_cast<chrono::microseconds>(end_time - start_time).count() <<
        " us"
        << std::endl;
    auto g_score = astar_result.first;
    auto path = astar_result.second;

    vector<pair<Position, int>> temp_result_list;

    // construct the path from start to end.
    auto current_index = end_index;
    if (path[current_index] == -1)
    {
        return PlanningResult();
    }
    while(path[current_index] != -1)
    {
        temp_result_list.push_back(graph.index2position(current_index));
        current_index = path[current_index];
    }
    temp_result_list.push_back(graph.index2position(start_index));
    // reverse the path 
    std::reverse(temp_result_list.begin(), temp_result_list.end());

    PlanningResult result;

    path_interpolation(result, temp_result_list, action_set, loc2pos_scale);

    
    result.path_length = g_score[end_index] * loc2pos_scale;
    result.is_complete = 1;
    auto end_time2 = chrono::high_resolution_clock::now();
        std::cout << "Astar+interpolation time consumed: " << 
        chrono::duration_cast<chrono::microseconds>(end_time2 - start_time).count() <<
        " us"
        << std::endl;
    return result;
}
