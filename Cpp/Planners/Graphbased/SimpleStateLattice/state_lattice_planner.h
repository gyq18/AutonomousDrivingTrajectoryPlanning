/*
 * @Author: Shengyong Li
 * @Date: 2022-04-21 13:07:46
 * @LastEditTime: 2022-04-23 11:03:14
 * @LastEditors: Please set LastEditors
 * @Description: State Lattice Planner header file,
 */

#pragma once
#include "state_lattice.h"
#include "json.hh"

/**
 * @class: LUT 
 * @brief: Class of look up table
 */
class LUT
{
public:
    int edge_length;
    std::vector<std::vector<double>> lut;
    int direction_num = 8;

    /**
     * @description: Load the lut data from file.
     * @param {string} filename.
     * @return {*}
     */
    void load_LUT(std::string filename);

    /**
     * @description: look up in lut.
     * @param {int} start_direction. The start direction. The start position is
     * set to (0, 0) 
     * @param {Position} to_pos. The destination position.
     * @param {int} to_direction. The expected direction when reaching tht destination.
     * @return The way length from start to end.
     */
    double look_up(int start_direction, Position to_pos, int to_direction) const;


};

class HLUTCallable {
public:
    ::LUT lut;
    HLUTCallable(std::string lut_file) { this->lut.load_LUT(lut_file);}
    double call(const StateLatticeGraph &, Position, int, Position, int);
};

class PlanningResult {
public:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    double path_length;
    int is_complete = 0;
};


/**
 * @description: Convert theta(rad) to direction.
 * The direction is an integer of 0~7.
 * The calculation method is defined below. 
 * @param {double} theta. 
 * @return The direction of 0~7.
 */
int theta2direction(double theta);

/**
 * @description: Convert direction to radian.
 * @param {int} direction. 0~7.
 * @return the radian theta.
 */
double direction2theta(int direction);


/**
 * @description: Convert absolute location to position in graph.
 * @param {Vec2d} loc. The absolute location.
 * @param {double} scale. The scale between graph and real environment.
 * @return The position in graph.
 */
Position loc2pos(math::Vec2d loc, double scale);

/**
 * @description: Convert position in graph to real location.
 * @param {Position} pos. The position in graph.
 * @param {double} scale. The scale between graph and real environment.
 * @return The real location.
 */
math::Vec2d pos2loc(Position pos, double scale);
math::Vec2d pos2loc(math::Vec2d pos, double scale);

/**
 * @description: A* search method implementation.
 * @param {const StateLatticeGraph&} graph. The graph that the seach algorithm will be applied in.
 * @param {Position} start_pos. The start position.
 * @param {int} start_direction. The start direction of the vehicle.
 * @param {Position} end_pos. The desitination position.
 * @param {int} end_direction. The direction expected in end_pos.
 * @param {std::function<double(const StateLatticeGraph &, Position, int, Position, int)>} heuristic_func. 
 * The heuristic fucntion. In order to use lambda function in this param, we set it to be std::function<>.
 * @param {std::function<bool(const StateLatticeGraph &, Position, int, Position, int)>} pos_validate.
 * This function check whether the move is validate.
 * @return pair of actual cost of each node and the vector that record the previous node of each node in the min cast way.
 */
std::pair<std::vector<double>, std::vector<NodeIndex>> Astar(const StateLatticeGraph & graph, 
    Position start_pos, int start_direction, Position end_pos, int end_direction,
    std::function<double(const StateLatticeGraph &, Position, int, Position, int)> heuristic_func,
    std::function<bool(const StateLatticeGraph &, Position, int, Position, int)> pos_validate
);


/**
 * @description: Check whether the move is valid.
 * @param {const StateLatticeGraph &} graph. The planning graph.
 * @param {Position} from_pos. The start position.
 * @param {int} from_direction. The start direction of 0~7.
 * @param {Position} to_pos. The destination position.
 * @param {int} to_direction. The destination direction of 0~7.
 * @param {double} loc2pos_scale. The scale between location(real coordinate) to position(relative coordinate.)
 * @return true if move is valid else false.
 */
bool check_move(const StateLatticeGraph & graph, Position from_pos, 
    int from_direction, Position to_pos, int to_direction, double loc2pos_scale);


/**
 * @description: HLUT heuristic function.
 * @param {LUT &} lut. The ref to an LUT instance.
 * @param {StateLatticeGraph &} graph. The StateLattice graph.
 * @param {Position} start_pos. The start position.
 * @param {int} start_direction. THe start direction.
 * @param {Position} end_pos. The end position. Usually the destination.
 * @param {int} end_direction. The end direction. 
 * @return The estimated cost from start_pos to end_pos.
 */
double hlut(const ::LUT & lut, const StateLatticeGraph & graph, Position start_pos, int start_direction, Position end_pos, int end_direction);


/**
 * @description: The state lattice planning method. The target is defined in global variables.
 * @param {double} loc2pos_scale. The scale between location(real coordinate) to position(relative coordinate).
 * @param {string} lut_file. The lut filename.
 * @param {string} action_file. The action filename.
 * @param {string} insert_points_file. The insert_points filename.
 * @return The planning result.
 */
PlanningResult SimpleStateLatticePlan(double loc2pos_scale, std::string lut_file, std::string action_file, std::string insert_points_file);
