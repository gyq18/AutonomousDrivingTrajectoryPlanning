"""
Reference: Informed RRT*: Optimal Sampling-based Path planning Focused via
Direct Sampling of an Admissible Ellipsoidal Heuristic
https://arxiv.org/pdf/1404.2334.pdf
"""
import time
import math
import random
import numpy as np
import globalvar


class InformedRRTStar:
    def __init__(self, start, goal, obstacleList, planning_scale_, expandDis=5, goal_sample_rate=10, maxIter=200):
        """
        :param start: the starting position of trajectory
        :param goal: the end point of trajectory
        :param obstacleList: globalvar.obstacles
        :param planning_scale_: globalvar.planning_scale_
        :param expandDis: the distance of expansion in every step, adjustment for planning_scale_
        :param goal_sample_rate: the probability of directly expanding to the end
        :param maxIter: the maximum times of iterations, adjustment for the complexity of the graph
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_randx = planning_scale_.xmin
        self.min_randy = planning_scale_.ymin
        self.max_randx = planning_scale_.xmax
        self.max_randy = planning_scale_.ymax
        self.expand_dis = expandDis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        self.node_list = None

    def informed_rrt_star_search(self):
        """
        use informed_rrt_star method to search for path
        :return: path and the length of path
        """
        self.node_list = [self.start]
        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        cBest = float('inf')
        solutionSet = set()
        path = None
        # Computing the sampling space
        cMin = math.sqrt(pow(self.start.x - self.goal.x, 2)
                         + pow(self.start.y - self.goal.y, 2))
        xCenter = np.array([[(self.start.x + self.goal.x) / 2.0],
                            [(self.start.y + self.goal.y) / 2.0], [0]])
        a1 = np.array([[(self.goal.x - self.start.x) / cMin],
                       [(self.goal.y - self.start.y) / cMin], [0]])
        e_theta = math.atan2(a1[1], a1[0])
        C = np.array([[math.cos(e_theta), -math.sin(e_theta), 0],
                      [math.sin(e_theta), -math.cos(e_theta), 0],
                      [0,                 0,                  1]])

        for i in range(self.max_iter):
            # Sample space is defined by cBest
            # cMin is the minimum distance between the start point and the goal
            # xCenter is the midpoint between the start and the goal
            # cBest changes when a new path is found
            rnd = self.informed_sample(cBest, cMin, xCenter, C)
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearestNode = self.node_list[n_ind]
            # steer
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.get_new_node(theta, n_ind, nearestNode)
            noCollision = self.check_move(nearestNode.x, nearestNode.y, newNode.x, newNode.y)

            if noCollision:
                # start_time = time.time()
                nearInds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearInds)
                self.node_list.append(newNode)

                d_goal = math.sqrt((newNode.x - self.goal.x) ** 2 + (newNode.y - self.goal.y) ** 2)
                if d_goal < self.expand_dis:
                    if self.check_move2(newNode.x, newNode.y, self.goal.x, self.goal.y):
                        solutionSet.add(newNode)
                        lastIndex = len(self.node_list) - 1
                        tempPathLen = newNode.cost + d_goal
                        if tempPathLen < cBest:
                            path = self.get_final_course(lastIndex)
                            cBest = tempPathLen
                            # print(cBest)
                # time_consumption = time.time() - start_time
                # print(time_consumption)

        return path, cBest

    def choose_parent(self, newNode, nearInds):
        """
        choose parent for the newNode
        :param newNode: the newest legal Node
        :param nearInds: index for the near nodes of newNode
        :return: the parent-changed newNode
        """
        if len(nearInds) == 0:
            return newNode

        dList = []
        for i in nearInds:
            near_node = self.node_list[i]
            if self.check_move2(newNode.x, newNode.y, near_node.x, near_node.y):
                d = math.hypot(newNode.x - near_node.x, newNode.y - near_node.y)
                dList.append(near_node.cost + d)
            else:
                dList.append(float('inf'))
        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("min cost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    def find_near_nodes(self, newNode):
        """
        find near nodes of newNode
        :param newNode: the newest legal Node
        :return: index for the near nodes of newNode
        """
        n_node = len(self.node_list)
        r = 40 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    def informed_sample(self, cMax, cMin, xCenter, C):
        """
        generate new points in an ellipse
        :param cMax: long axis of the ellipse——length of the shortest path which has been searched
        :param cMin: focal length of the ellipse——the Euclidean distance between start and end
        :param xCenter: the center of the ellipse
        :param C: transformation matrix
        :return: a new point in the ellipse
        """
        if cMax < float('inf'):
            r = [cMax / 2.0,
                 math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
                 0]
            L = np.diag(r)
            xBall = self.sample_unit_ball()
            rnd = np.dot(np.dot(C, L), xBall) + xCenter
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sample_free_space()
        return rnd

    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b), b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    def sample_free_space(self):
        """
        generate a random point when no path has been searched
        :return: a random new point
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_randx, self.max_randx),
                   random.uniform(self.min_randy, self.max_randy)]
        else:
            rnd = [self.goal.x, self.goal.y]
        return rnd

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        """
        for the new point, find the nearest Node in node_list
        :param nodes: self.node_list
        :param rnd: the new point in graph
        :return: index for the nearest Node
        """
        dList = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        """
        generate a new Node based on the new point
        :param theta: the angle from the nearestNode to newNode
        :param n_ind: index for the nearestNode—— parent of the newNode
        :param nearestNode: the nearest Node in node_list to the new point
        :return: a new Node
        """
        from copy import deepcopy
        newNode = deepcopy(nearestNode)

        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)

        newNode.cost += self.expand_dis
        newNode.parent = n_ind
        return newNode

    def check_move(self, x1, y1, x2, y2):
        """
        A simple function to judge whether there are obstacles between (x1, y1) to (x2, y2).
        :param x1, y1: the start position
        :param x2, y2: the end position
        :return: If move is valid, return True else False.
        """
        from main_unstructure import checkObj_linev, checkObj_point
        obstacles = self.obstacle_list
        for obs in obstacles[0]:
            if checkObj_point(np.array([x2, y2]), np.vstack([obs.x, obs.y])):
                return False

            if checkObj_linev(np.array([x1, y1]), np.array([x2, y2]), np.vstack([obs.x, obs.y])):
                return False

        return True

    def check_move2(self, x1, y1, x2, y2):
        """
        another check_move method when (x2, y2) is known legal
        each parameters are the same as check_move
        """
        from main_unstructure import checkObj_linev
        obstacles = self.obstacle_list
        for obs in obstacles[0]:
            if checkObj_linev(np.array([x1, y1]), np.array([x2, y2]), np.vstack([obs.x, obs.y])):
                return False

        return True

    def get_final_course(self, lastIndex):
        """
        get a path through the parent of Nodes
        :param lastIndex: the index for the last node in node_list, which is actually the parent of the end.
        :return: path
        """
        path = [[self.goal.x, self.goal.y]]
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def ResamplePathWithEqualDistance(x, y, theta, num_nodes_s):
    """
    resample path with equal distance——make path have enough nodes
    :param x, y, theta: initial x, y & theta
    :param num_nodes_s: the number of nodes in path
    :return: path(x, y, theta) which divided in equal distance by num_nodes_s nodes
    """
    x_new = []
    y_new = []
    theta_new = []
    for i in range(1, len(theta)):
        while theta[i] - theta[i - 1] > math.pi:
            theta[i] = theta[i] - 2 * math.pi
        while theta[i] - theta[i - 1] < -math.pi:
            theta[i] = theta[i] + 2 * math.pi
    x_extended = []
    y_extended = []
    theta_extended = []
    for j in range(len(x) - 1):
        distance = math.hypot(x[j + 1] - x[j], y[j + 1] - y[j])
        LARGE_NUM = round(distance * 100)
        for k in range(LARGE_NUM):
            x_extended.append(x[j] + k * (x[j + 1] - x[j]) / LARGE_NUM)
            y_extended.append(y[j] + k * (y[j + 1] - y[j]) / LARGE_NUM)
            theta_extended.append(theta[j] + k * (theta[j + 1] - theta[j]) / LARGE_NUM)
    x_extended.append(x[-1])
    y_extended.append(y[-1])
    theta_extended.append(theta[-1])
    for ii in range(num_nodes_s - 1):
        index = round(ii * len(x_extended) / (num_nodes_s - 1))
        x_new.append(x_extended[index])
        y_new.append(y_extended[index])
        theta_new.append(theta_extended[index])
    x_new.append(x_extended[-1])
    y_new.append(y_extended[-1])
    theta_new.append(theta_extended[-1])
    return x_new, y_new, theta_new





def main():
    from main_unstructure import GenerateStaticObstacles_unstructured, VisualizeDynamicResults
    from TransformPathToTrajectory import TransformPathToTrajectory
    globalvar.Nobs = 20
    globalvar.obstacles_ = GenerateStaticObstacles_unstructured()
    vehicle_TPBV_ = globalvar.vehicle_TPBV_

    print("Start informed rrt star planning")

    # Set params
    rrt = InformedRRTStar(start=[vehicle_TPBV_.x0, vehicle_TPBV_.y0],
                          goal=[vehicle_TPBV_.xtf, vehicle_TPBV_.ytf],
                          planning_scale_=globalvar.planning_scale_, obstacleList=globalvar.obstacles_, maxIter=200)
    start_time = time.time()
    path, path_length = rrt.informed_rrt_star_search()
    time_consumption = time.time() - start_time
    print(f"InformedRRTStarPlan time consumption: {time_consumption}")
    # print(path)
    x = []
    y = []
    theta = []
    len_path = len(path)
    for i in range(len_path):
        x.append(path[len_path - i - 1][0])
        y.append(path[len_path - i - 1][1])
    # print(x, y)
    theta.append(vehicle_TPBV_.theta0)
    for j in range(1, len_path):
        theta.append(math.atan2(y[j] - y[j - 1], x[j] - x[j - 1]))
    # print(theta)

    transmethod_flag = 1
    x, y, theta = ResamplePathWithEqualDistance(x, y, theta, globalvar.num_nodes_s)
    print(x, y, theta)
    trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag)
    VisualizeDynamicResults(trajectory)


if __name__ == '__main__':
    main()
