#! ./venv/bin/python
from functools import partial
import heapq

import numpy as np
import globalvar
from state_lattice import StateLatticeGraph, ActionSet
from main_unstructure import GenerateStaticObstacles_unstructured, VisualizeDynamicResults, \
    checkObj_linev, checkObj_point
from TransformPathToTrajectory import TransformPathToTrajectory
import json

DEBUG = False

class LUT:
    """
        look up table 
        use load_LUT(...) to load file from disk.
        use look_up(...) to get the value
    """
    def __init__(self) -> None:
        self.lut = None
        self.edge_length = None
        self.direction_num = 8

    def load_LUT(self, filename: str):
        """
            load lookup table file from disk
        Args:
            filename: str, filename of the lut file

        """
        f = open(filename, 'r', encoding='utf8')
        json_str = f.read()
        record = json.loads(json_str)
        edge_length = record['edge_length']

        look_up_table = []
        for direction in range(8):
            look_up_table.append(record[f"{direction}"])
        
        self.lut, self.edge_length = np.array(look_up_table), edge_length

    def look_up(self, start_direction, to_pos, to_direction):
        """
            get the value of specific pos and direction.
        Args:
            start_direction: integer in 0~7 which refers to theta of direction * pi/4,
                It's the initial direction of the vehicle.
            to_pos: tuple of (int, int). position of the destination. 
                x, y must in [-1, 0, 1].
            to_direction: integer in 0~7, the expect direction(or theta) when car reach destination.
        
        Returns:
            the expected cost of the given action.
        
        All these params control the moving method of the vehicle.

        """
        # default start at pos (0, 0)
        x, y = to_pos
        assert x < self.edge_length and y < self.edge_length, f"x: {x}, y: {y}"
        index = x * self.edge_length * self.direction_num + y * self.direction_num + to_direction

        return self.lut[start_direction, index]


def theta2direction(theta):
    """
        convert theta to direction:
    Args:
        theta: float, the degree of direction in radian.

    Returns:
        the direction of 0~7
    """
    return np.round(theta / (np.pi / 4)).astype(int) % 8


def direction2theta(direction):
    """
        convert direction to theta:
    Args:
        theta: float, the degree of direction in radian.

    Returns:
        theta in radian.
    """
    return direction * np.pi / 4


def loc2pos(loc, scale):
    """
        Convert location to position. In this context, loc means absolute position 
        in environment, pos means the relative position in graph.
    Args:
        loc: (float, float). The absolute position in planning_scale_
        scale: float. the scaling factor of graph and planning_scale_

    Returns:
        the pos if type of (int , int)
    """
    planning_scale_ = globalvar.planning_scale_
    x_min = planning_scale_.xmin
    y_min = planning_scale_.ymin
    return int((loc[0] - x_min) / scale), int((loc[1] - y_min)/scale)


def pos2loc(pos, scale):
    """
        Convert position to location. In this context, loc means absolute position 
        in environment, pos means the relative position in graph.
    Args:
        pos: (int, int). The absolute position in planning_scale_
        scale: float. the scaling factor of graph and planning_scale_

    Returns:
        the loc if type of (float , float)
    """
    pos = np.array(pos)
    planning_scale_ = globalvar.planning_scale_
    x_min = planning_scale_.xmin
    y_min = planning_scale_.ymin
    if len(pos.shape) == 1:
        return pos[0]* scale + x_min, scale * pos[1] + y_min
    else:
        pos[:, 0] = pos[:, 0] * scale + x_min
        pos[:, 1] = pos[:, 1] * scale + y_min
        return pos


def Astar(graph: StateLatticeGraph, start_pos, start_direction, end_pos, end_direction, heuristic_func, pos_validate):
    """
        A* search method. This function will give a possible way from start to end. The problem is defined 
        in an searh problem in StateLatticeGraph.
    Args:
        graph: instance of StateLatticeGraph. Graph contains necessary information for calculation.
        start_pos: (int, int). The start position of the vehicle. Must inside the graph.
        start_direction: integer of 0~7. The start direction of the vehicle.
        end_pos: (int, int). The destination.
        end_direction: integer of 0~7. The direction when finish moving.
        heuristic_func: python function. This function will be called like 
            heuristic_func(graph, adj_pos, adj_direction, end_pos, end_direction)
        pos_validate: python function. This function will be called like
            pos_validate(graph, current_pos, current_direction, other_pos, other_direction):
        
    Returns:
        (g_score, path)
        g_score: list[int]. The actual cost of the point. Each index of g_score refers to a specific (pos, direction).
            The pos ans direction can be extract by calling graph.index2position(index) 
        path: dict. This dict contains the node index and its predecessor in the best way.

    """
    nodes = graph.nodes
    edges = graph.edges

    start_index = graph.position2index(start_pos, start_direction)
    end_index = graph.position2index(end_pos, end_direction)

    # start A*
    close_index = []
    open_list = []
    path = {}

    # initialize the start node, calculate f = g + h
    g_score = np.full((len(nodes), ), float('inf'))
    g_score[start_index] = 0

    h_start = heuristic_func(graph, start_pos, start_direction, end_pos, end_direction)
    f_start = g_score[start_index] + h_start
    heapq.heappush(open_list, (f_start, start_index))

    while len(open_list) != 0:
        # get the node with the minimum f, move it to close_index
        current_f, current_index = heapq.heappop(open_list)
        current_pos, current_direction = graph.index2position(current_index)
        close_index.append(current_index)

        # if end_index in open_list and it has the best f score,
        # we can end the searching procedure
        if current_index == end_index:
            break

        for adj_index, edge_cost in edges[current_index]:
            if adj_index in close_index:
                continue

            # if there is obstacle in edge, it means the adj_index is actually
            # not a adjacent node of current node. So we continue and ignore it.
            adj_pos, adj_direction = graph.index2position(adj_index)
            if not pos_validate(graph, current_pos, current_direction, adj_pos, adj_direction):
                continue
            temp_g_score = g_score[current_index] + edge_cost
            if temp_g_score < g_score[adj_index]:
                path[adj_index] = current_index
                g_score[adj_index] = temp_g_score

                f_score = g_score[adj_index] + heuristic_func(graph, adj_pos, adj_direction, end_pos, end_direction)
                heapq.heappush(open_list, (f_score, adj_index))
        
    return g_score, path


def check_move(graph, from_pos, from_direction, to_pos, to_direction, scale=1):
    """
        A simple function to judge whether there are obstacles between from_pos 
        and to_pos.
    Args:
        graph: instance of StateLatticeGraph.
        from_pos: (int ,int). The start position.
        from_direction: integer of 0~7. the start direction.
        to_pos: (int, int). The ending position.
        to_direction: integer of 0~7. The ending direction.
        scale: float. The scaling factor of planning_scale_ and position inside the graph.
    
    Returns:
        If move is valid, return True else False.
    """
    obstacles = globalvar.obstacles_

    from_loc = pos2loc(from_pos, scale)
    to_loc = pos2loc(to_pos, scale)

    for obs in obstacles[0]:
        if checkObj_point(np.array(to_loc), np.vstack([obs.x, obs.y])):
            return False
    
        if checkObj_linev(np.array(from_loc), np.array(to_loc), np.vstack([obs.x, obs.y])):
            return False
    
    return True


def hlut(lut, graph, from_pos, from_direction, to_pos, to_direction):
    """
        The Heuristic LookUp Table. An heuristic fuction implemented with lookup table.
    Args:
        lut: instance of LUT. The lookup table with params loaded.
        graph: instance of StateLatticeGraph.
        from_pos: (int ,int). The start position.
        from_direction: integer of 0~7. the start direction.
        to_pos: (int, int). The ending position.
        to_direction: integer of 0~7. The ending direction.
    
    Returns:
        The value of the guessing cost from (from_pos, from_direction) to (to_pos, to_direction)
    """
    edge_length = lut.edge_length
    delta_x, delta_y = np.array(to_pos) - np.array(from_pos)
    
    # if actual graph is bigger than lut,
    # we use an overflow factor to describe how bigger it is,
    # the result will be multiplied by this factor

    # because lut can not process negitive position, we 
    # need to reconstuct an euqal move which has positive position
    if delta_x < 0:
        delta_x = - delta_x
        # reflect with y-axis
        from_direction = ((4 - from_direction) + 8) % 8
        to_direction = ((4 - to_direction) + 8) % 8
    if delta_y < 0:
        delta_y = - delta_y
        # reflect with x-axis
        from_direction = ((- from_direction) + 8) % 8
        to_direction = ((- to_direction) + 8) % 8

    # if move is smaller than lut, then directly return the result
    if delta_x < edge_length and delta_y < edge_length:
        return lut.look_up(from_direction, (delta_x, delta_y), to_direction)
    
    # else we calculate the overload_factor, delta_x and delta_y
    if delta_x < edge_length and delta_y >= edge_length:
        overflow_factor = (delta_y + 1) / edge_length
        delta_x = np.round(delta_x * edge_length / (delta_y + 1)).astype(int)
        delta_y = edge_length - 1

    elif delta_x >= edge_length and delta_y < edge_length:
        overflow_factor = (delta_x + 1) / edge_length
        delta_y = np.round(delta_y * edge_length / (delta_x + 1)).astype(int)
        delta_x = edge_length - 1
    else:
        overflow_factor = (delta_x + 1) / (delta_y + 1) if delta_x > delta_y else (delta_y + 1) / (delta_x + 1)
        delta_x = min(np.round(edge_length * (delta_x + 1) / (delta_y + 1)).astype(int), edge_length - 1)
        delta_y = min(np.round(edge_length * (delta_y + 1) / (delta_x + 1)).astype(int), edge_length - 1)
    
    # return the estimation result.
    return overflow_factor * lut.look_up(from_direction, (delta_x, delta_y), to_direction)


def ImprovedStateLatticePlan(loc2pos_scale=1, lutfile="./look_up_table_121628.txt"):
    """
        Improved State Lattice Planner. This function will give
        a possible trajectory from start to end.
    Args:
        loc2pos_scale: float. Refers to edge scale between graph and actual map.

    Returns:
        [x, y, theta, path_length, is_complete] as defined above.
    """

    # initalize global variables
    obstacles = globalvar.obstacles_
    vehicle_TPBV_ = globalvar.vehicle_TPBV_
    planning_scale_ = globalvar.planning_scale_
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    vehicle_kinematics_ = globalvar.vehicle_kinematics_

    # process scaling. this will define how big is the graph.
    graph_width = planning_scale_.x_scale
    graph_height = planning_scale_.y_scale
    graph = StateLatticeGraph(int(graph_width / loc2pos_scale), int(graph_height / loc2pos_scale))

    # get the start location and the end location.
    start_loc, start_direction = (vehicle_TPBV_.x0, vehicle_TPBV_.y0), theta2direction(vehicle_TPBV_.theta0)
    end_loc, end_direction = (vehicle_TPBV_.xtf, vehicle_TPBV_.ytf), theta2direction(vehicle_TPBV_.thetatf)

    # convert location to position in graph.
    start_pos = loc2pos(start_loc, loc2pos_scale)
    end_pos = loc2pos(end_loc, loc2pos_scale)

    end_index = graph.position2index(end_pos, end_direction)
    start_index = graph.position2index(start_pos, start_direction)

    # initial lookup table
    lut = LUT()
    lut.load_LUT(lutfile)
 
    # start A* fucntion 
    start_time = time.time()
    g_score, path = Astar(graph, start_pos, start_direction, end_pos, end_direction, partial(hlut, lut), partial(check_move, scale=loc2pos_scale))
    time_consumption = time.time() - start_time
    print(f"Astar time comsumption: {time_consumption}")

    # reconstruct the minimum cost path.
    result = []
    i = end_index
    if DEBUG:
        print(f"{graph.nodes[i]}-{g_score[i]}", end='')
    result.append((graph.nodes[i].position, graph.nodes[i].direction))
    if i in path:
        current = path[i]
        while current in path:
            n = graph.nodes[current]
            if DEBUG:
                print(f' <- {n}-{g_score[current]}', end='')
            result.append((n.position, n.direction))
            current = path[current]
        if DEBUG:
            print(f' <- {graph.nodes[start_index]}-{g_score[start_index]}', end='')
        result.append((start_pos, start_direction))
    else:
        # if we can not go to the destination, just return 0 of is_complete.
        return [[0], [0], [0], 0, 0]
    print('')

    result.reverse()
    xy_list = np.array([r[0] for r in result])
    direction_list = np.array([r[1] for r in result])
    
    # The result is attained but it is represented only by the lattice vertex
    # in the graph. It is discrete and unfriendly to human.
    # We need to add more points in the path to make it smoother.
    xy_full = []
    theta_full = []
    for i in range(len(xy_list)-1):
        # ActionSet.sample_points can give more points base on the action
        # it will automatically extract action from pos.
        xy_points, theta_points = ActionSet.sample_points(xy_list[i], direction_list[i], xy_list[i+1], direction_list[i+1])
        xy_full.extend(xy_points)
        theta_full.extend(theta_points)
    xy_full_loc = pos2loc(xy_full, loc2pos_scale)
    
    return [xy_full_loc[:, 0], xy_full_loc[:, 1], theta_full, 
        g_score[end_index] * loc2pos_scale, 1] 


if __name__ == "__main__":
    import time

    globalvar.Nobs = 0

    globalvar.obstacles_ = GenerateStaticObstacles_unstructured()
    # print(globalvar.vehicle_kinematics_.min_turning_radius)
    # print(globalvar.vehicle_geometrics_.r2x)
    
    start_time = time.time()
    x, y, theta, path_length, is_complete = ImprovedStateLatticePlan(1, "./look_up_table_121628.txt")
    # print(theta)
    # print(len(x), len(y), len(theta), path_length)
    time_consumption = time.time() - start_time
    print(f"ImprovedStateLatticePlan time comsumption: {time_consumption}")
    transmethod_flag=1 # choose different path to trajectory method
    trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag)

    VisualizeDynamicResults(trajectory)

    global obs


