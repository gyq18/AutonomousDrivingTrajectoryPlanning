from math import ceil, fabs
from cv2 import waitKey
import numpy as np
import sortedcontainers
import globalvar
import cv2
from TransformPathToTrajectory import TransformPathToTrajectory
from main_unstructure import checkObj_linev, checkObj_point, CreateVehiclePolygon, distance, \
    GenerateStaticObstacles_unstructured, VisualizeStaticResults, VisualizeDynamicResults


def calculate_xy_to_index(x,y):
    idx_x = ceil((x-globalvar.planning_scale_.xmin)/globalvar.resolution_x)
    idx_y = ceil((y-globalvar.planning_scale_.ymin)/globalvar.resolution_y)
    if idx_x < 0:
        idx_x = 0
    if idx_x >= globalvar.num_nodes_x:
        idx_x = globalvar.num_nodes_x - 1
    if idx_y < 0:
        idx_y = 0
    if idx_y >= globalvar.num_nodes_y:
        idx_y = globalvar.num_nodes_y - 1
    return idx_x,idx_y


def CreateCostmap():
    cost_ndarr = np.zeros((globalvar.num_nodes_x,globalvar.num_nodes_y,1),dtype=np.uint8)
    fill_list = []
    for obstacle in globalvar.obstacles_[0]:
        obs = np.vstack((obstacle.x, obstacle.y))
        obs_idx = []
        for col in range(obs.shape[1]):
            idx_x,idx_y = calculate_xy_to_index(obs[0,col],obs[1,col])
            obs_idx.append((idx_x,idx_y))
        fill_list.append(np.array(obs_idx))
    cv2.fillPoly(cost_ndarr,np.array(fill_list),255)
    

    length_unit = 0.5*(globalvar.resolution_x+globalvar.resolution_y)
    radius = globalvar.vehicle_geometrics_.radius/length_unit
    radius = int(radius)
    elemEllipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(radius,radius))
    costmap_ = cv2.dilate(cost_ndarr,elemEllipse)
    return costmap_



def IsVertexValid(x,y):
    x_inf = globalvar.planning_scale_.xmin
    x_sup = globalvar.planning_scale_.xmax
    y_inf = globalvar.planning_scale_.ymin
    y_sup = globalvar.planning_scale_.ymax
    if x> x_sup or x<x_inf or y>y_sup or y<y_inf:
        return False
    idx_x, idx_y = calculate_xy_to_index(x,y)
    if globalvar.costmap_[idx_x][idx_y] == 255:
        return False
    return True

def Is3DNodeValid(x,y,theta):
    xr = x + globalvar.vehicle_geometrics_.r2x * np.cos(theta)
    yr = y + globalvar.vehicle_geometrics_.r2x * np.sin(theta)
    xf = x + globalvar.vehicle_geometrics_.f2x * np.cos(theta)
    yf = y + globalvar.vehicle_geometrics_.f2x * np.sin(theta)
    x_inf = globalvar.planning_scale_.xmin + globalvar.vehicle_geometrics_.radius * 1.01
    x_sup = globalvar.planning_scale_.xmax - globalvar.vehicle_geometrics_.radius * 1.01
    y_inf = globalvar.planning_scale_.ymin + globalvar.vehicle_geometrics_.radius * 1.01
    y_sup = globalvar.planning_scale_.ymax - globalvar.vehicle_geometrics_.radius * 1.01
    if xr > x_sup or xr < x_inf or xf > x_sup or xf < x_inf or yr > y_sup or yr < y_inf or yf > y_sup or yf < y_inf:
        return False
    idx_r_x, idx_r_y = calculate_xy_to_index(xr,yr)
    idx_f_x, idx_f_y = calculate_xy_to_index(xf,yf)
    if globalvar.costmap_[idx_r_y,idx_r_x]==255 or globalvar.costmap_[idx_f_y,idx_f_x]==255:
        return False
    return True


# check feasible edge when produced
def IsLineValid(p1, p2):  # p1(x,y),p2(x,y) represent a vector p1----->p2
    theta = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
    V1 = CreateVehiclePolygon(x=p1[0], y=p1[1], theta=theta)
    V2 = CreateVehiclePolygon(x=p2[0], y=p2[1], theta=theta)
    X = np.hstack((V1.x, V2.x))
    Y = np.hstack((V1.y, V2.y))
    min_x_index = np.argmin(X)
    max_x_index = np.argmax(X)
    min_y_index = np.argmin(Y)
    max_y_index = np.argmax(Y)
    min_x_v = np.min(X)
    max_x_v = np.max(X)
    min_y_v = np.min(Y)
    max_y_v = np.max(Y)
    P1 = np.array([X[min_x_index], Y[min_x_index]])
    P2 = np.array([X[max_x_index], Y[max_x_index]])
    P3 = np.array([X[min_y_index], Y[min_y_index]])
    P4 = np.array([X[max_y_index], Y[max_y_index]])
    for obstacle in globalvar.obstacles_[0]:

        obs = np.vstack((obstacle.x, obstacle.y))
        min_x_obs = np.min(obstacle.x)
        max_x_obs = np.max(obstacle.x)
        min_y_obs = np.min(obstacle.y)
        max_y_obs = np.max(obstacle.y)
        if (min_x_obs >= min_x_v and max_x_obs <= max_x_v and min_y_obs >= min_y_v and max_y_obs <= max_y_v) or \
                (min_x_obs <= min_x_v and max_x_obs >= max_x_v and min_y_obs <= min_y_v and max_y_obs >= max_y_v):
            return False
        if checkObj_linev(P1, P4, obs) or checkObj_linev(P4, P2, obs) or checkObj_linev(P2, P3, obs) or checkObj_linev(
                P3, P1, obs):
            return False

    return True


def getvertex(k):
    vertex = np.array([globalvar.vehicle_TPBV_.x0, globalvar.vehicle_TPBV_.y0])
    vertex = np.vstack((vertex, [globalvar.vehicle_TPBV_.xtf, globalvar.vehicle_TPBV_.ytf]))
    lx = globalvar.planning_scale_.xmin
    ux = globalvar.planning_scale_.xmax
    ly = globalvar.planning_scale_.ymin
    uy = globalvar.planning_scale_.ymax

    while vertex.shape[0] < k + 2:
        x = (ux - lx) * np.random.rand() + lx
        y = (uy - ly) * np.random.rand() + ly
        pt = np.array([x, y])
        if not IsVertexValid(x,y):
            continue
        vertex = np.vstack((vertex, pt))
    return vertex


def get_edges(vertex):
    size = vertex.shape[0]
    edges = []
    for i in range(size):
        edges.append([])
    for i in range(size):
        for j in range(i + 1, size):
            p1 = vertex[i, :]
            p2 = vertex[j, :]
            dist = np.linalg.norm(p1-p2)
            x_samp = np.linspace(p1[0],p2[0],num=ceil(dist/(0.1*globalvar.vehicle_geometrics_.radius)))
            y_samp = np.linspace(p1[1],p2[1],num=ceil(dist/(0.1*globalvar.vehicle_geometrics_.radius)))
            theta_ = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
            IsfeasibleLine = 1
            for x, y in zip(x_samp,y_samp):
                if not Is3DNodeValid(x,y,theta_):
                    IsfeasibleLine = 0
                    break
            if IsfeasibleLine:
                edges[i].append(j)
                edges[j].append(i)
    return edges


class PriorityQueue(object):

    def __init__(self, node):
        self._queue = sortedcontainers.SortedList([node])

    def push(self, node):
        self._queue.add(node)

    def pop(self):
        return self._queue.pop(index=0)

    def empty(self):
        return len(self._queue) == 0

    def compare_and_replace(self, i, node):
        if node < self._queue[i]:
            self._queue.pop(index=i)
            self._queue.add(node)

    def find(self, node):
        try:
            loc = self._queue.index(node)
            return loc
        except ValueError:
            return None


class Node(object):
    def __init__(self, vertex_order=0, g=0, h=0, parent=None):
        self.vertex_order = vertex_order
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.vertex_order == other.vertex_order


def resample(path):
    for i in range(len(path) - 1):
        dis = distance(path[i], path[i + 1])
        if i == 0:
            obj_x = np.linspace(path[i][0], path[i + 1][0], round(dis))
            obj_x = np.delete(obj_x, -1)
            obj_y = np.linspace(path[i][1], path[i + 1][1], round(dis))
            obj_y = np.delete(obj_y, -1)
        elif i == len(path) - 2:
            obj_x = np.hstack((obj_x, np.linspace(path[i][0], path[i + 1][0], round(dis))))
            obj_y = np.hstack((obj_y, np.linspace(path[i][1], path[i + 1][1], round(dis))))
        else:
            obj_x = np.hstack((obj_x, np.linspace(path[i][0], path[i + 1][0], round(dis))))
            obj_x = np.delete(obj_x, -1)
            obj_y = np.hstack((obj_y, np.linspace(path[i][1], path[i + 1][1], round(dis))))
            obj_y = np.delete(obj_y, -1)
    path = np.vstack((obj_x, obj_y)).T
    return path


def PRM():
    vertex = getvertex(100)

    print('get vertex over')
    edges = get_edges(vertex)
    print('get edges over')
    node = Node(h=distance(vertex[0], vertex[1]))
    completeness_flag = 0
    path_length = 0
    iter_num = 0
    max_iter = 500
    open_list = PriorityQueue(node)
    closed = set()
    path = []
    prev = None
    while (not open_list.empty()) and (not completeness_flag) and (iter_num < max_iter):
        cur_node = open_list.pop()
        closed.add(cur_node.vertex_order)
        for edge in range(len(edges[cur_node.vertex_order])):
            newVertex = edges[cur_node.vertex_order][edge]
            child_g = cur_node.g + distance(vertex[cur_node.vertex_order], vertex[newVertex])
            child_h = distance(vertex[1], vertex[newVertex])
            child_node = Node(vertex_order=newVertex, g=child_g, h=child_h, parent=cur_node)
            if newVertex not in closed and not open_list.find(child_node):
                prev = child_node
                if child_node.vertex_order == 1:
                    path_length = child_node.f
                    completeness_flag = 1
                    break
                open_list.push(child_node)
        iter_num += 1

    while prev is not None:
        curp = vertex[prev.vertex_order]
        path.append(curp)
        prev = prev.parent
    path = list(reversed(path))
    if completeness_flag:
        path = resample(path)
        print('succeed')
    else:
        print('failed')
        return
    x = path[:, 0]
    y = path[:, 1]
    theta = []
    for i in range(len(path) - 1):
        theta.append(np.arctan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0]))
    theta.append(0)
    theta = np.array(theta)
    return x, y, theta, path_length, completeness_flag


if __name__ == "__main__":
    globalvar.obstacles_ = GenerateStaticObstacles_unstructured()
    globalvar.costmap_  = CreateCostmap()

    [x, y, theta, path_length, completeness_flag] = PRM()
    transmethod_flag = 1  # choose different path to trajectory method
    trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag)
    VisualizeStaticResults(trajectory)
    VisualizeDynamicResults(trajectory)



