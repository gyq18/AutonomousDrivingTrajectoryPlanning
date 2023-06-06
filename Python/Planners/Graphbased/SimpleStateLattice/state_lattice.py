#! ./venv/bin/python
import numpy as np
import heapq
import copy


def action2cost(direction, action):
    """
        Calculate the edge cost.
    Args:
        direction: interger of 0~7. The start direction.
        action: (int, int, int, int). Action defined in ActionSet.

    Returns:
        the cost of the action, which is actually edge cost.
    """
    delta_x, delta_y, next_direction, is_forward = action
    # punishment for moving backward
    coff = 1 if is_forward else 1
    if direction == next_direction:
        # if direction stay unchange, use Euclidean distance
        return np.hypot(delta_x, delta_y) * coff
    else:
        # if direction changed, use 1/4 perimeter of circle
        return np.pi / 2 * coff


class ActionSet:
    """
        The action set. This class record all allowed action in this state lattice problem.
        The action is in format of (delta_x, delta_y, next_direction, is_forward).

        direction_actions_0 record all possible action when start_direction is 0.
        direction_actions_1 record all possible action when start_direction is 1.
        All action from other start_direction(2~7) can be attained by rotation.

        action_functions are function that describe the moving curve of actions.
        Each function is record as (independent variable, curve function, differential of the curve function)
        We use curve function to calculate curve, use differential of the curve function to calculate
        theta.

        action_functions_0 record all functions when start_direction is 0.
        action_functions_1 record all functions when start_direction is 1.
        All the result from other start_direction(2~7) can be attained by rotation.
    """
    
    # Set of possible actions, in format of (delta_x, delta_y, next_direction, is_forward)
    # the first line of action stands for moving forward, the second for moving backward
    direction_actions_0 = [(1, 0, 0, 1), (1, 1, 1, 1), (1, 1, 2, 1), (1, -1, 6, 1), (1, -1, 7, 1)] + \
            [(-1, 0, 0, 0), (-1, 1, 6, 0), (-1, 1, 7, 0), (-1, -1, 1, 0), (-1, -1, 2, 0)]
    direction_actions_1 = [(1, 1, 0, 1), (1, 1, 1, 1), (1, 1, 2, 1), (0, 1, 3, 1), (1, 0, 7, 1)] + \
        [(-1, -1, 0, 0), (-1, -1, 1, 0), (-1, -1, 2, 0), (0, -1, 3, 0), (-1, 0, 7, 0)]
    action_functions_0 = [
        ('x', '0*x', '0*x'),
        ('x', '-x**3+2*x**2', '-3*x**2+4*x'),
        ('x', ' - np.sqrt(1 - x**2) + 1', 'x / (np.sqrt(1-x**2)+1e-6)'),
        ('x', 'np.sqrt(1 - x**2) - 1', '-x / (np.sqrt(1-x**2)+1e-6)'),
        ('x', 'x**3-2*x**2', '3*x**2-4*x'),
        ('x', '0*x', '0*x'),
        ('x', ' - np.sqrt(1 - x**2) + 1', 'x / (np.sqrt(1-x**2)+1e-6)'),
        ('x', 'x**3+2*x**2', '3*x**2+4*x'),
        ('x', '-x**3-2*x**2', '-3*x**2-4*x'),
        ('x', 'np.sqrt(1 - x**2) - 1', '-x / (np.sqrt(1-x**2)+1e-6)'),
        
    ]

    action_functions_1 = [
        ('x', '-x**3+x**2+x', '-3*x**2+2*x+1'),
        ('x', 'x', '1+0*x'),
        ('y', '-y**3+y**2+y', '-3*y**2+2*y+1'),
        ('y', 'np.sqrt(1/2 - (y-1/2)**2)-1/2', '-(y - 0.5)/(np.sqrt(0.5-(y-0.5)**2))'),
        ('x', 'np.sqrt(1/2 - (x-1/2)**2)-1/2', '(-(x - 0.5))/(np.sqrt(0.5-(x-0.5)**2))'),
        ('x', '-x**3-x**2+x', '-3*x**2-2*x+1'),
        ('x', 'x', '1+0*x'),
        ('y', '-y**3-y**2+y', '(-3*y**2-2*y+1)'),
        ('y', '-np.sqrt(1/2 - (y+1/2)**2)+1/2', '(y+0.5)/(np.sqrt(0.5-(y+0.5)**2))'),
        ('x', '-np.sqrt(1/2 - (x+1/2)**2)+1/2', '((x + 0.5))/(np.sqrt(0.5-(x+0.5)**2) + 1e-6)'),
    ]


    @staticmethod
    def to_base_action(start_direction, end_direction, delta_x, delta_y):
        """
            Calculate the base action(action when start direction is 0 or 1) from action.
        Args:
            start_pos: (int, int). The start position of the vehicle. Must inside the graph.
            start_direction: integer of 0~7. The start direction of the vehicle.
            end_pos: (int, int). The destination.
            end_direction: integer of 0~7. The direction when finish moving.

        Returns:
            the base action(action when start direction is 0 or 1)
        """
        assert delta_x in (-1, 0, 1) and delta_y in (-1, 0, 1) and \
            start_direction in range(8) and end_direction in range(8), \
                f"invalid action, {start_direction}, {end_direction}, {delta_x}, {delta_y}"
        base_action = None
        # rotate action to direction = 0 or direction = 1
        if start_direction % 2 == 0:
            base_action = ActionSet.action_rotate(((8-start_direction) % 8) // 2, (delta_x, delta_y, end_direction, 1))
        else:
            base_action = ActionSet.action_rotate(((9-start_direction) % 8) // 2, (delta_x, delta_y, end_direction, 1))

        # judge if the action is forward or not
        is_forward = 1
        if start_direction % 2 == 0:
            if base_action[0] < 0:
                is_forward = 0
        else:
            if base_action[0] < 0 or base_action[1] < 0:
                is_forward = 0

        base_action = (base_action[0], base_action[1], base_action[2], is_forward)
        return base_action




    @staticmethod
    def sample_points(start_pos, start_direction, end_pos, end_direction, points_num=10):
        """
            Add more points to specific action. Because the path is describe by position and direction,
            it is not smooth at all. This function use action_functions to add more points
            to actions. 
        Args:
            start_pos: (int, int). The start position of the vehicle. Must inside the graph.
            start_direction: integer of 0~7. The start direction of the vehicle.
            end_pos: (int, int). The destination.
            end_direction: integer of 0~7. The direction when finish moving.
            points_num: number of points to sample.

        Returns:
            (p, t). p is an list of points in (x, y), theta is the corresponding theta.
        """
        delta_x, delta_y = end_pos[0] - start_pos[0], end_pos[1] - start_pos[1]

        # calculate the rotate information
        rotate_num = start_direction // 2
        set_num = start_direction % 2
        action_index = None
        # get the base action
        base_action = ActionSet.to_base_action(start_direction, end_direction, delta_x, delta_y)
        assert base_action[0] in (-1, 0, 1) and base_action[1] in (-1, 0, 1)
        if set_num == 0:
            direction_action_set = ActionSet.direction_actions_0
            func_set = ActionSet.action_functions_0
        else:
            direction_action_set = ActionSet.direction_actions_1
            func_set = ActionSet.action_functions_1
        action_index = direction_action_set.index(base_action)
        assert action_index != -1, 'can not find action_index'
        func_record = func_set[action_index]

        # use the base action to sample more points.
        x = None
        y = None
        theta = None
        if func_record[0] == 'x':
            func = lambda x: eval(func_record[1])
            diff_func = lambda x: eval(func_record[2])
            if base_action[0] == -1:
                x = np.linspace(0, -1, points_num)
            else: 
                x = np.linspace(0, 1, points_num)
            y = func(x)
            theta = np.arctan2(diff_func(x), 1)
        elif func_record[0] == 'y':
            func = lambda y: eval(func_record[1])
            diff_func = lambda y: eval(func_record[2])
            if base_action[1] == -1:
                y = np.linspace(0, -1, points_num)
            else: 
                y = np.linspace(0, 1, points_num)
            x = func(y)
            theta = (np.pi / 2 - np.arctan2(diff_func(y), 1) + np.pi * 2) % (np.pi * 2)
        
        # rotate from the base action to original action
        pos = np.vstack([x, y]).T
        rot_matrix = np.array([[0, 1], [-1, 0]])
        for _ in range(rotate_num):
            pos = np.matmul(pos, rot_matrix)
            theta = (theta + np.pi / 2) % (2 * np.pi)
        
        pos[:, 0] += start_pos[0]
        pos[:, 1] += start_pos[1]
        return pos, theta


        

    
    @staticmethod
    def action_rotate(rotate_num, direction_action):
        """
            Rotate an action of rotate_num * pi / 2.
        Args:
            rotate_num: int. number of pi / 2 to ratate.
            direction_action: tuple. Defined in ActionSet.
        
        Returns:
            the new_action rotated.
        """
        direction_max = 8
        new_action = list(copy.copy(direction_action))
        for _ in range(rotate_num):
            new_action[0], new_action[1], new_action[2] = \
                - new_action[1], new_action[0], (new_action[2] + 2) % direction_max

        return new_action

    @staticmethod
    def action_clean(direction_actions, position, x_max, y_max):
        """
            Delete the illegal actions in direction_actions.
        Args:
            direction_actions: list of actions.
            position: (int, int). The current position in graph.
            x_max, y_max: int. They define the size of the graph. The graph min 
                pos is (0, 0)

        Returns:
            An list of legal actions.
        """
        temp_actions = []
        for action in direction_actions:
            # del invalid action when x is on the edge of lattice
            if position[0] == 0:
                if action[0] == -1:
                    continue
                elif action[0] == 0 and action[2] in [1, 7]:
                    continue
            elif position[0] == x_max - 1:
                if action[0] == 1:
                    continue
                elif action[0] == 0 and action[2] in [3, 5]:
                    continue
            
            # del invalid action when y is on the edge of lattice
            if position[1] == 0:
                if action[1] == -1:
                    continue
                elif action[1] == 0 and action[2] in [1, 3]:
                    continue
            elif position[1] == y_max - 1:
                if action[1] == 1:
                    continue
                elif action[1] == 0 and action[2] in [4, 7]:
                    continue
            temp_actions.append(action)
        
        return temp_actions

    @staticmethod
    def get_actions(pos, direction, x_length, y_length):
        """
            Get the posible action in pos and direction.
        Args:
            position: (int, int). The current position.
            direction: integer of 0~7. The current direction.
            x_length, y_length: They define the size of the graph. The graph min 
                pos is (0, 0)
        
        """
        result_action = None
        direction_actions_0 = copy.copy(ActionSet.direction_actions_0)
        direction_actions_1 = copy.copy(ActionSet.direction_actions_1)

        if direction == 0:
            result_action = direction_actions_0
        elif direction == 1:
            result_action = direction_actions_1
        else:
            rotate_num = direction // 2
            if direction % 2 == 0:
                result_action = [ActionSet.action_rotate(rotate_num, action) for action in direction_actions_0]
            else:
                result_action = [ActionSet.action_rotate(rotate_num, action) for action in direction_actions_1]
            
        return ActionSet.action_clean(result_action, pos, x_length, y_length)


class LatticeNode:
    """
        Lattice node class. It save some infomation of a lattice.
    """
    def __init__(self, position, direction) -> None:
        self.position = position

        # in the simplest situation, each point has only one
        # direction state which is similar with "theta" in car's 
        # property. But direction is an integer, 0 for 0 degree,
        # 1 for 45 degress, 2 for 90 degree and so on. 
        self.direction = direction

        # self.cost = init_cost
    
    def __str__(self):
        return f'LatticeNode(position=({self.position[0]}, {self.position[1]}), direction={self.direction})'

    def __lt__(self, other):
        return self.cost < other.cost


class StateLatticeGraph:
    '''
        The State Lattice Graph. We use an graph to describe the state lattice and solve 
        problems on it.

    '''
    def __init__(self, x_edge_length, y_edge_length) -> None:
        '''
        Args:
            x_edge_length: int. x_axis length, start at 0.
            y_edge_length: int. y_axis length, start at 0.
        '''
        self.x_length = x_edge_length
        self.y_length = y_edge_length
        self.direction_num = 8
        
        self.node_num = self.x_length * self.y_length * self.direction_num

        self.nodes, self.edges = self.build_graph()
        self.dsearch_called = False

    def index2position(self, index):
        '''
            Convert index representation to pos, direction representation.

        Args:
            index: int. The index of the node.
        
        Returns:
            (x, y), direction
        '''
        x = index // (self.y_length * self.direction_num)
        y = (index - x * self.y_length * self.direction_num) // self.direction_num
        direction = index - x * self.y_length * self.direction_num - y * self.direction_num
        return (x, y), direction

    def position2index(self, position, direction):
        '''
            Convert position, direction representation to index representation.

        Args:
            position: (int, int). The position of the node.
            direction: integer of 0~7. The direction of the node.

        Returns:
            the index of the node.
        '''
        x, y = position
        return x * self.y_length * self.direction_num + y * self.direction_num + direction


    

    
    def build_graph(self):
        """
            Add node and edge to graph.

        Returns:
            graph_nodes, and edge. Edges is dict{index: list[(adj_index, edge_cost)]}
                graph_nodes is a list of LatticeNode
        """
        # add nodes
        graph_nodes = []
        for x in range(self.x_length):
            for y in range(self.y_length):
                for direction in range(self.direction_num):
                    graph_nodes.append(LatticeNode((x, y), direction))

        # add edges, edges is dict{index: list[(adj_index, edge_cost)]}
        edges = {}
        for x in range(self.x_length):
            for y in range(self.y_length):
                for direction in range(self.direction_num):
                    # process if node is at the edge
                    direction_actions = ActionSet.get_actions((x, y), direction, self.x_length, self.y_length)
                    # calculate index.
                    index_i = x * self.y_length * self.direction_num + y * self.direction_num + direction
                    next_state_index = []
                    
                    # each action correspond to an edge
                    for action in direction_actions:
                        next_x, next_y, next_direction = x + action[0], y + action[1], action[2]

                        index_j = next_x * self.y_length * self.direction_num + next_y * self.direction_num + next_direction
                        edge_cost = action2cost(direction, action)
                        next_state_index.append((index_j, edge_cost))
                    edges[index_i] = next_state_index
        
        return graph_nodes, edges
                
    def dijkstra_search(self, start_index):
        '''
            Dijkstra algorithm. 
        
        Args:
            start_index: int. The index of a node. Searching will start at
                that node.
        
        Returns:
            g_score: list of float, record the cost to each node. It use the same 
                index as graph_nodes.
            path: dict. This dict contains the node index and its predecessor in the best way.
        '''
        if self.dsearch_called:
            self.nodes, self.edges = self.build_graph()
        self.dsearch_called = True
        graph_nodes = self.nodes
        edges = self.edges

        close_index = []
        open_list = []
        path = {}

        g_score = np.full((len(graph_nodes), ), float('inf'))
        g_score[start_index] = 0
        heapq.heappush(open_list, (g_score[start_index], start_index))


        while len(open_list) != 0:
            # get the minimum cost node in open_list
            current_cost, current_index = heapq.heappop(open_list)
            # move node to close
            close_index.append(current_index)

            # check adjacent node and update cost
            for adj_index, edge_cost in edges[current_index]:
                adj_node = graph_nodes[adj_index]

                if g_score[adj_index] > current_cost + edge_cost:
                    path[adj_index] = current_index
                    g_score[adj_index] = current_cost + edge_cost
                    heapq.heappush(open_list, (g_score[adj_index], adj_index))
        return g_score, path
    


if __name__ == "__main__":
    import json
    state_lattice = StateLatticeGraph(40, 40)
    import matplotlib.pyplot as plt
    f_action = open("actions.txt", 'w')
    f_insert_points = open("insert_points.txt", 'w')
    action_dict = {}
    insert_points_dict = {}
    for direction in range(8):
        res = ActionSet.get_actions((1, 1), direction, 40, 40)
        action_dict[direction] = res
        print(direction)
        plt.xlim([-1.2, 1.2])
        plt.ylim([-1.2, 1.2])
        for i, action in enumerate(res):
            print(action)
            pos, theta = ActionSet.sample_points((0, 0), direction, action[0:2], action[2], 20)
            x = pos[:, 0]
            y = pos[:, 1]
            plt.plot(x, y)
            
            if direction not in insert_points_dict:
                insert_points_dict[direction] = {}
            insert_points_dict[direction][i] = {
                'x': list(x),
                'y': list(y),
                'theta': list(theta)
            }
            # plt.show()
        # plt.show()
    f_action.write(json.dumps(action_dict))
    f_insert_points.write(json.dumps(insert_points_dict))
    