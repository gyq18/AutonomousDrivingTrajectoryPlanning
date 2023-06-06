import numpy as np

class Obstacle:
    def __init__(self, x, y):
        self.x = np.asarray(x)
        self.y = np.asarray(y)

    def point_at(self, i):
        return np.asarray([self.x[i], self.y[i]])   


def GenerateDynamicObstacles_unstructured(config):
    st_graph_search_ = config.st_graph_search_
    Nobs_dyn = 5

    start_obs = []
    end_obs = []

    start_obs.append(Obstacle([10,15,15,10], [10,10,15,15]))
    end_obs.append(Obstacle(  [45,50,50,45], [10,10,15,15]))

    start_obs.append(Obstacle([45,50,50,45], [10,10,15,15]))
    end_obs.append(Obstacle(  [45,50,50,45], [45,45,50,50]))

    start_obs.append(Obstacle([45,50,50,45], [45,45,50,50]))
    end_obs.append(Obstacle(  [10,15,15,10], [45,45,50,50]))

    start_obs.append(Obstacle([10,15,15,10], [45,45,50,50]))
    end_obs.append(Obstacle(  [10,15,15,10], [10,10,15,15]))

    start_obs.append(Obstacle([17,19,25,23], [19,17,23,25]))
    end_obs.append(Obstacle(  [34,36,42,40], [36,34,40,42]))

    obstacle_map = []
    for i in range(Nobs_dyn):
        dx = end_obs[i].x - start_obs[i].x
        dy = end_obs[i].y - start_obs[i].y

        obstacle_map.append([])
        for j in range(st_graph_search_.num_nodes_t):
            temp_obs = Obstacle(
                start_obs[i].x + dx / st_graph_search_.num_nodes_t * j, 
                start_obs[i].y + dy / st_graph_search_.num_nodes_t * j
                )
            obstacle_map[i].append(temp_obs)
    
    return obstacle_map


if __name__ == "__main__":
    from types import SimpleNamespace
    st_graph_search_ = SimpleNamespace(num_nodes_t=10)

    obs_map = GenerateDynamicObstacles_unstructured()

    print(obs_map)
    print(np.asarray(obs_map).shape)