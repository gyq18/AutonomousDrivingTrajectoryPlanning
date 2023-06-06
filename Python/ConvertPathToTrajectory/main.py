# import global_config first
from  global_config import config

from types import SimpleNamespace
from statelatticeplanner.state_lattice_planner import SimpleStateLatticePlan
from statelatticeplanner.main_unstructure import GenerateStaticObstacles_unstructured
from GenerateDynamicObstacles_unstructured import GenerateDynamicObstacles_unstructured
from PlanSpeedForDynamicScenarios import PlanSpeedForDynamicScenarios, VisualizeDynamicResultsForDynamicScenarios
import matplotlib
# matplotlib.use('agg')

## genearate random static obstacle
# global obstacles_ Nobs 
# global margin_obs_ # margin_obs_ for dilated obstacles
margin_obs_=0.5
config.margin_obs_ = margin_obs_
Nobs = 0
config.Nobs = Nobs
obstacles_ = GenerateStaticObstacles_unstructured(config)
config.obstacles_ = obstacles_
# global costmap_

x, y, theta, path_length, is_complete = SimpleStateLatticePlan(config, 1, "statelatticeplanner/look_up_table_153244.txt")

#%% the next is mainly for plan speed for dynamic obstacles


num_nodes_s=60


# st_graph settings
st_graph_search_ = SimpleNamespace()
st_graph_search_.k = 2
st_graph_search_.num_nodes_s = num_nodes_s
st_graph_search_.num_nodes_t = st_graph_search_.k*num_nodes_s
st_graph_search_.multiplier_H_for_A_star = 2.0
st_graph_search_.penalty_for_inf_velocity = 4
st_graph_search_.max_t = round(path_length * 2)
st_graph_search_.max_s = path_length
st_graph_search_.resolution_s = path_length / st_graph_search_.num_nodes_s
st_graph_search_.resolution_t = st_graph_search_.max_t / st_graph_search_.num_nodes_t
config.st_graph_search_ = st_graph_search_

dynamic_obs =  GenerateDynamicObstacles_unstructured(config)
config.dynamic_obs = dynamic_obs

trajectory = PlanSpeedForDynamicScenarios(x, y, theta, config)

VisualizeDynamicResultsForDynamicScenarios(trajectory, config)
