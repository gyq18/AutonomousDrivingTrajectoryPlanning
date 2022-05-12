import time
from state_lattice import StateLatticeGraph
import json



class LookupTableGenerator:
    """
        Lookup table generator. This class use StateLatticeGraph and dijkstra 
        algorithm to generate lookup table
    """
    def __init__(self, edge_length) -> None:
        """
        Args:
            edge_length: edge's length in state lattice graph
        """
        self.edge_length = edge_length
        self.direction_num = 8

        self.graph = StateLatticeGraph(self.edge_length, self.edge_length)
        
    
    def save(self, filename=f'look_up_table_{time.strftime("%H%M%S")}.txt'):
        """
            Save lookup table data into a file.
        Args:
            filename: str, filename of the file you want to save to.

            The file is save in json format.
            key "edge_length" saves the edge length of the graph.

            The other data is the data of an direction and keys from 0 to 7. The start pos is default of (0, 0).
            In each key of lut, the data is aligned in x_axis, y_axis, direction_axis.
            For example, if you want to find the cost from start_direction=2 to pos (x, y), end_direction,
            you need to go to key 3(because direction start from 0), and find the number in **index** of
            x * y_axis_len * direction_axis_len + y * direction_axis_len + end_direction.
            In this problem the direction_axis_len is pinned to 8. y_axis_len is the edge_length.
        """
        f = open(filename, 'w')
        # print(f"{self.edge_length}", file=f)
        # print(f"{self.direction_num}", file=f)
        result_dict = {}
        result_dict['edge_length'] = self.edge_length
        for direction in range(self.direction_num):
            g_score, _ = self.graph.dijkstra_search(direction)
            # calculated_distance = [f"{score:.4f}" for score in g_score]
            # print(', '.join(calculated_distance), file=f)
            result_dict[f"{direction}"] = list(g_score)
        json_obj = json.dumps(result_dict)
        f.write(json_obj)





if __name__ == "__main__":
    # direction = 6
    # state_lattice = StateLatticeGraph(40, 40)
    #
    # graph_nodes, edges = state_lattice.nodes, state_lattice.edges
    #
    # g_score, path = state_lattice.dijkstra_search(direction)
    #
    # # for y in range(2, -1, -1):
    # #     for x in range(3):
    # #         for direction in range(8):
    # #             print(x * 24 + y * 8 + direction, end=' ')
    # #         print(" # ", end='')
    # #     print()
    # # for node in graph_nodes:
    # #     print(node)
    #
    #
    # import pprint
    # # pprint.pprint(edges)
    # # pprint.pprint(graph)
    # # print(path)
    #
    # for i in [13, 14, 15]:
    #
    #     print(f"{graph_nodes[i]}-{g_score[i]}", end='')
    #     if i in path:
    #         current = path[i]
    #         while current in path:
    #             print(f' <- {graph_nodes[current]}-{g_score[current]}', end='')
    #             current = path[current]
    #         if current == direction:
    #             print(f' <- {graph_nodes[direction]}-{g_score[current]}', end='')
    #     print('')

    lut_generator = LookupTableGenerator(60)
    lut_generator.save()

