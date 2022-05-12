/*
 * @Author: Lishengyong
 * @Date: 2022-04-20 12:58:06
 * @LastEditTime: 2022-04-23 10:29:33
 * @LastEditors: Please set LastEditors
 * @Description: state lattice base header file
 */

#pragma once
#include "vec2d.h"
#include <vector>
#include "json.hh"
#include <string>

using namespace math;
using json = nlohmann::json;


typedef int NodeIndex;

// Position class
// inner properties "x" and "y" are integer.
// This class represent the position in a StateLatticeGraph
class Position
{

protected:
    int x_;
    int y_;

public:
    Position(int x, int y) : x_(x), y_(y) {}
    Position() : x_(0), y_(0) {}
    int x() const {return x_;} 
    int y() const {return y_;}
    void set_x(int x) {this->x_ = x;}
    void set_y(int y) {this->y_ = y;}

    /**
     * @description: operator override.
     * @param other an other Position instance. 
     * @return Position calculated
     */    
    Position operator-(const Position &other) const
    {
        return Position(this->x_ - other.x(), this->y_ - other.y());
    }

};


// ActionSet class
// This class save possible action the vehicle may take.
// This class also record date to insert to more points to trajectory.
class ActionSet
{
public:
    json actions;
    json sample_points_list;

    /**
     * @description: 
     * @param std::string action_file. The file that saves all possible actions.
     * @param std::string insert_points_file. The points data to make trajectory more smooth.
     */
    void load_data(std::string action_file, std::string insert_points_file);
    
    /**
     * @description: Get all possible action in a specific position and direction.
     * @param Position position. The current position. 
     * @param int direction. The current direction. interger of 0~7.
     * @param int x_length. The x-axis length of the graph.
     * @param int y_length. The y-axis length of the graph.
     * @return vector of actions. An action is defined as a 4-element vector.
     */    
    std::vector<std::vector<int> >  get_actions(Position position, int direction, int x_length, int y_length);

    /**
     * @description: sample more points for an action, make the trajectory smooth.
     * @param Position start_pos. The start position.
     * @param int start_direction. The start direction.
     * @param Position end_pos. The end position.
     * @param Position end_direction. The end direction.
     * @return A pair, the first element is the pair of x and y, the second element is theta.
     * Element x, y and theta are vector<double>
     */    
    std::pair<std::pair<std::vector<double>, std::vector<double> >, std::vector<double> > 
        sample_points(Position start_pos, int start_direction, Position end_pos, int end_direction);

    /**
     * @description: 
     * @param vector<std::vector<int>>& action_list. The action list. Usually returned by ActionSet::get_actions .
     * @param Position position. The current position.
     * @param int x_len. The size of the graph. Decide the border of the graph.
     * @param int y_len. The size of the graph. Decide the border of the graph.
     * @return The allowed actions in specific position.
     */    
    std::vector<std::vector<int> > action_clean(const std::vector<std::vector<int> > &action_list, Position position, int x_len, int y_len);
    

};

/** 
 * @class: LatticeNode
 * @brief: Node in StateLatticeGraph
 */
class LatticeNode {
public:
    Position position;
    int direction;
    LatticeNode(Position p, int a) noexcept: position(p), direction(a) {};
    LatticeNode() noexcept: position(Position(0, 0)), direction(0) {};
};

/** 
 * @class: LatticeEdge
 * @brief: Edge in StateLatticeGraph
 */
class LatticeEdge {
public:
    NodeIndex index;
    int cost;
    LatticeEdge(NodeIndex index, int cost) noexcept: index(index), cost(cost) {};
    LatticeEdge() noexcept: index(0), cost(0) {};
};

/** 
 * @class: StateLatticeGraph
 * @brief: Graph data structure of StateLattice planning
 */
class StateLatticeGraph {
public:
    int x_length;
    int y_length;
    int direction_num;
    int node_num;
    std::vector<LatticeNode> nodes;
    std::vector<std::vector<LatticeEdge>> edges;
    ActionSet action_set;


public:
    StateLatticeGraph(int x_len, int y_len, const ActionSet & action_set) noexcept;

    /**
     * @description: Convert index represenctation to (position, direction) representation.
     * @param {NodeIndex} index. The node index
     * @return Pair of the node position and node direction.
     */    
    std::pair<Position, int> index2position(NodeIndex index) const;

    /**
     * @description: Convert (position, direction) representation to index represenctation.
     * @param {Position} position. The node position.
     * @param {int} direction. The node direction.
     * @return The index of the node in the graph.
     */    
    NodeIndex position2index(Position position, int direction) const;

    /**
     * @description: Build the graph. Full in the nodes and edges.
     */    
    void build_graph();
    


};