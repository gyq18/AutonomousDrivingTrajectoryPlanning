import math

import matplotlib.pyplot as plt

import numpy as np

import cvxopt
from cvxopt import matrix

import time
from numpy.linalg import norm as norm

import globalvar

import scipy
# from scipy.optimize import minimize

from globalvar import planning_scale_, vehicle_geometrics_, vehicle_kinematics_, vehicle_TPBV_, Nobs, margin_obs_, hybrid_astar_, planning_scale_, hybrid_astar_, costmap_


def quadprog(H, f, L, k, Aeq, beq, lb, ub, z0):
    """
    Input: Numpy arrays, the format follows MATLAB quadprog function: https://www.mathworks.com/help/optim/ug/quadprog.html
    Output: Numpy array of the solution
    """
    n_var = H.shape[1]

    P = cvxopt.matrix(H, tc='d')
    q = cvxopt.matrix(f, tc='d')

    L = np.concatenate([L, np.eye(n_var), -np.eye(n_var)], axis=0)
    k = np.concatenate([k, ub, -lb], axis=0)

    L = cvxopt.matrix(L, tc='d')
    k = cvxopt.matrix(k, tc='d')

    Aeq = cvxopt.matrix(Aeq, tc='d')
    beq = cvxopt.matrix(beq, tc='d')

    cvxopt.solvers.options['show_progress'] = False
    # sol = cvxopt.solvers.qp(P, q, L, k, Aeq, beq, initvals = z0)
    sol = cvxopt.solvers.qp(P, q, L, k, Aeq, beq)
    z1 = np.array(sol['x'])
    return z1, sol['status']


class trajectoryclass:
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.theta = None
        self.v = None
        self.a = None
        self.phi = None
        self.omega = None


def kinematiccon(x, nstep, Apdyn, Aqdyn, Athetadyn, Lm):
    theta = x[2 * nstep: 3 * nstep] * 1
    vel = x[3 * nstep: 4 * nstep] * 1
    phi = x[4 * nstep: 5 * nstep] * 1
    Ts = x[5 * nstep] / (nstep - 1)
    ceq = np.concatenate([np.dot(Apdyn, x[0:2 * nstep]) - np.multiply(Ts * vel[0:nstep - 1], np.cos(theta[0:nstep - 1])),
                         np.dot(Aqdyn, x[0:2 * nstep]) - np.multiply(Ts *
                                                                     vel[0:nstep - 1], np.sin(theta[0:nstep - 1])),
                         np.dot(Athetadyn, theta[0:nstep]) - np.multiply(Ts / Lm * vel[0:nstep - 1], np.tan(phi[0:nstep - 1]))], axis=0)
    return ceq


def ComputeJac(x, Lm):
    num_dv = len(x)
    nstep = round((num_dv - 1) / 5)
    delta_t = x[num_dv - 1] / (nstep - 1)
    Jac = np.zeros((3 * nstep - 3, num_dv))

    for kkk in range(nstep - 1):
        # index1 = [2 * kkk - 1, 2 * kkk + 1, 2 * nstep + kkk, 3 * nstep + kkk, num_dv]
        # Jac(kkk, index1) = [-1, 1, delta_t * x(3 * nstep + kkk) * sin(x(2 * nstep + kkk)), -delta_t * cos(x(2 * nstep + kkk)), -x(3 * nstep + kkk) * cos(x(2 * nstep + kkk)) / (nstep - 1)]
        Jac[kkk][2 * kkk] = -1
        Jac[kkk][2 * kkk + 2] = 1
        Jac[kkk][2 * nstep + kkk] = delta_t * \
            x[3 * nstep + kkk] * math.sin(x[2 * nstep + kkk])
        Jac[kkk][3 * nstep + kkk] = - delta_t * math.cos(x[2 * nstep + kkk])
        Jac[kkk][num_dv - 1] = -x[3 * nstep + kkk] * \
            math.cos(x[2 * nstep + kkk]) / (nstep - 1)
        # index2 = [2 * kkk, 2 * kkk + 2, 2 * nstep + kkk, 3 * nstep + kkk, num_dv]
        # Jac(nstep - 1 + kkk, index2) = [-1, 1, -delta_t * x(3 * nstep + kkk) * cos(x(2 * nstep + kkk)), -delta_t * sin(x(2 * nstep + kkk)), -x(3 * nstep + kkk) * sin(x(2 * nstep + kkk)) / (nstep - 1)]
        Jac[nstep - 1 + kkk][2 * kkk + 1] = -1
        Jac[nstep - 1 + kkk][2 * kkk + 3] = 1
        Jac[nstep - 1 + kkk][2 * nstep + kkk] = - delta_t * \
            x[3 * nstep + kkk] * math.cos(x[2 * nstep + kkk])
        Jac[nstep - 1 + kkk][3 * nstep + kkk] = - \
            delta_t * math.sin(x[2 * nstep + kkk])
        Jac[nstep - 1 + kkk][num_dv - 1] = -x[3 * nstep + kkk] * \
            math.sin(x[2 * nstep + kkk]) / (nstep - 1)
        # index3 = [nstep * 2 + kkk, nstep * 2 + kkk + 1, 3 * nstep + kkk, 4 * nstep + kkk, num_dv]
        # Jac(2 * nstep - 2 + kkk, index3) = [-1, 1, -delta_t * tan(x(4 * nstep + kkk)) / Lm,
        # -delta_t * x(3 * nstep + kkk) / (cos(x(4 * nstep + kkk))^2 * Lm),
        # -x(3 * nstep + kkk) * tan(x(4 * nstep + kkk)) / ((nstep - 1) * Lm)]
        Jac[2 * nstep - 2 + kkk][nstep * 2 + kkk] = -1
        Jac[2 * nstep - 2 + kkk][nstep * 2 + kkk + 1] = 1
        Jac[2 * nstep - 2 + kkk][3 * nstep + kkk] = - \
            delta_t * math.tan(x[4 * nstep + kkk]) / Lm
        Jac[2 * nstep - 2 + kkk][4 * nstep + kkk] = -delta_t * \
            x[3 * nstep + kkk] / (math.cos(x[4 * nstep + kkk])**2 * Lm)
        Jac[2 * nstep - 2 + kkk][num_dv - 1] = -x[3 * nstep + kkk] * \
            math.tan(x[4 * nstep + kkk]) / ((nstep - 1) * Lm)
    return Jac

# 第一种处理碰撞约束的方法：Box


def Find_Box_for_all_obj(xr):

    def IsPointValidInDilatedMap(x, y):
        is_valid = 0
        # global planning_scale_ hybrid_astar_ costmap_
        ind_x = math.floor((x - planning_scale_.xmin) /
                           hybrid_astar_.resolution_x) + 1
        ind_y = math.floor((y - planning_scale_.ymin) /
                           hybrid_astar_.resolution_y) + 1

        if ((ind_x < 0) or (ind_x >= hybrid_astar_.num_nodes_x) or (ind_y < 0) or (ind_y >= hybrid_astar_.num_nodes_y)):
            return is_valid

        if (costmap_[ind_x][ind_y] == 1):
            return is_valid

        is_valid = 1
        return is_valid

    def SpinTrial(xc, yc):

        if (IsPointValidInDilatedMap(xc, yc)):
            return xc, yc

        unit_length = hybrid_astar_.resolution_x * 0.5
        ii = 0

        while (1):
            ii = ii + 1
            angle_type = ii % 8
            radius = 1 + (ii - angle_type) / 8
            angle = angle_type * math.pi / 4
            x_nudge = xc + math.cos(angle) * radius * unit_length
            y_nudge = yc + math.sin(angle) * radius * unit_length

            if (IsPointValidInDilatedMap(x_nudge, y_nudge)):
                xc = x_nudge
                yc = y_nudge
                return xc, yc

        return xc, yc

    def IsCurrentExpansionValid(xc, yc, test, lb, ind):
        is_valid = 0

        ax = xc - lb[1]
        ay = yc + lb[0]
        bx = xc + lb[3]
        by = yc + lb[0]
        cx = xc + lb[3]
        cy = yc - lb[2]
        dx = xc - lb[1]
        dy = yc - lb[2]

        ds = hybrid_astar_.resolution_x * 0.5

        if(ind == 0):
            xmax = bx
            xmin = ax
            ymin = ay
            ymax = yc + test[0]
        elif(ind == 1):
            xmax = ax
            xmin = xc - test[1]
            ymin = dy
            ymax = ay
        elif(ind == 2):
            xmax = cx
            xmin = dx
            ymin = yc - test[2]
            ymax = cy
        elif(ind == 3):
            xmax = xc + test[3]
            xmin = cx
            ymin = cy
            ymax = by
        else:
            return is_valid

        if ((xmax > planning_scale_.xmax - vehicle_geometrics_.radius) or
                (xmin < planning_scale_.xmin + vehicle_geometrics_.radius) or
                (ymax > planning_scale_.ymax - vehicle_geometrics_.radius) or
                (ymin < planning_scale_.ymin + vehicle_geometrics_.radius)):
            return is_valid

        xx = np.array([])
        yy = np.array([])

        nx = math.ceil((xmax - xmin) / ds) + 1
        ny = math.ceil((ymax - ymin) / ds) + 1

        for x in range(int(xmin - 1), int(xmax), nx):

            for y in range(int(ymin - 1), int(ymax), ny):
                xx = np.concatenate([xx, np.array([x])])
                yy = np.concatenate([yy, np.array([y])])

        ind_x = np.floor((xx - planning_scale_.xmin) /
                         hybrid_astar_.resolution_x) + 1
        ind_y = np.floor((yy - planning_scale_.ymin) /
                         hybrid_astar_.resolution_y) + 1

        ind_x = ind_x.astype(int)
        ind_y = ind_y.astype(int)

        ind_x1 = ind_x.tolist()
        ind_y1 = ind_y.tolist()
        if (costmap_[ind_x1, ind_y1] == 1):
            return is_valid

        is_valid = 1
        return is_valid

    def GetAabbLength(xc, yc):
        ds = 0.5
        # 0.1
        smax = 5
        # 2

        lb = np.zeros(4)

        is_completed = np.zeros(4)

        while (np.sum(is_completed) < 4):

            for ind in range(4):

                if (is_completed[ind]):
                    continue

                test = lb * 1

                if (test[ind] + ds > smax):
                    is_completed[ind] = 1
                    continue

                test[ind] = test[ind] + ds

                if (IsCurrentExpansionValid(xc, yc, test, lb, ind)):
                    lb = test
                else:
                    is_completed[ind] = 1
            return lb

    # xr的大小2*1
    xc, yc = SpinTrial(xr[0][0], xr[1][0])
    lb = GetAabbLength(xc, yc)
    bxmax = max(xc + lb[3], xc - lb[1])
    bxmin = min(xc + lb[3], xc - lb[1])
    bymax = max(yc + lb[0], yc - lb[2])
    bymin = min(yc + lb[0], yc - lb[2])
    A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
    b = np.array([[bxmax], [-bxmin], [bymax], [-bymin]])

    return A, b


# 第二种处理碰撞约束的方法：CFS


def Find_CFS_for_all_obj(xr, obstacle):
    # 针对障碍物所有obj，为当前参考点xr寻找凸可行集CFS：F(xr)
    # xr的大小2*1，obj的大小2*4（每一列是一个顶点的坐标）
    # 首先确定函数phi：因为实例中的障碍物都是凸的，因此phi取为文献中的公式(23)
    # 1. 需要注意的是，公式（23）所示的是phi是凸函数，并且光滑，因此次梯度集合为单点集，只包含一个元素——梯度
    # 2. 此外我们需要寻找障碍物边界上距离xr最近的点：依次考虑各个相邻顶点
    ncorner = obstacle.shape[1]  # 障碍物顶点个数
    nobj = math.floor(obstacle.shape[0] / 2)

    d = 10000000 * np.ones((nobj, 1))  # xr到障碍物的最小距离初始值设为无穷大
    # 考虑所有的障碍物，得到的凸可行集(aTx<=b)保存在pre_A和pre_b中
    pre_A = np.zeros((nobj, 2))
    pre_b = np.zeros((nobj, 1))
    counter = 0
    index_true = np.ones((nobj, 1))  # index_true(j)=0表示排除了第j条直线
    single_A = np.array((1, 2))
    single_b = 0
    # 确定了函数phi之后，F(xr)为：phi(xr) + delta_phi(xr)*(x-xr)>=0
    for j in range(nobj):
        counter = counter + 1
        obj = obstacle[2 * j:2 * j + 2]

        for i in range(ncorner):
            corner1 = obj[:, i]
            corner1 = corner1.reshape(2, 1)
            corner2 = obj[:, (i + 1) % ncorner]
            corner2 = corner2.reshape(2, 1)
            # xr,corner1以及corner2三点构成三角形，求三边的长度
            dist_r1 = norm(xr - corner1)
            dist_r2 = norm(xr - corner2)
            dist_12 = norm(corner1 - corner2)
            # 若角r12为钝角，则此时xr距离corner1更近
            if (dist_r1**2 + dist_12**2 - dist_r2**2) < -1e-4:  # 余弦定理
                temp_d = dist_r1
                temp_A = xr.T - corner1.T
                temp_b = np.dot(temp_A, corner1)
            elif (dist_r2**2 + dist_12**2 - dist_r1**2) < -1e-4:  # 若角r21为钝角，则此时xr距离corner2更近
                temp_d = dist_r2
                temp_A = xr.T - corner2.T
                temp_b = np.dot(temp_A, corner2)
            else:  # 若角r12以及角r21均为锐角，则xr到由corner1和corner2构成的直线段的垂线最短
                project_length = np.dot(
                    (xr - corner1).T, (corner2 - corner1)) / dist_12
                temp_d = math.sqrt(dist_r1**2 - project_length**2)
                temp_A = np.array(
                    [[corner1[1][0] - corner2[1][0], corner2[0][0] - corner1[0][0]]])
                temp_b = np.dot(corner2[0], corner1[1]) - \
                    np.dot(corner1[0], corner2[1])

            if temp_d < d[j]:
                d[j] = temp_d * 1
                single_A = temp_A * 1
                single_b = temp_b * 1

        length_A = norm(single_A)
        single_A = single_A / length_A
        single_b = single_b / length_A

        # 离xr最近的顶点的对角点，与xr应该分居直线Ax=b的两侧
        for kkk in range(ncorner):
            tmp = (obj[:, kkk]).reshape(2, 1)
            if np.dot(single_A, tmp) < single_b:
                single_A = -single_A * 1
                single_b = -single_b * 1
                break
        pre_A[counter - 1, :] = single_A * 1
        pre_b[counter - 1, :] = single_b * 1
    # 看看是否pre_A和pre_b中是否有冗余约束
    # 在上面，每一个障碍物都对应找了一条直线分割xr和障碍物本身。如果排除这条直线，剩下的直线仍然可以分割这个障碍物和xr,则这条直线确实可以从约束中排除
    for j in range(nobj):
        temp_index = index_true * 1
        temp_index[j] = 0
        cur_index = np.where(temp_index > 0)
        cur_A = pre_A[cur_index[0], :] * 1
        cur_b = pre_b[cur_index[0]] * 1
        obj = obstacle[2 * j:2 * j + 2]
        result = np.dot(cur_A, obj) - np.tile(cur_b, (1, 4))

        flag = np.sum(result >= 0, axis=1)
        if ((flag[np.where(flag >= 4)] > 0).any()):
            index_true[j][0] = 0
    final_index = np.where(index_true > 0)
    A = pre_A[final_index[0], :]
    b = pre_b[final_index[0]]
    return A, b, d


def PlanOptimizationL1Trajectory(initialguess, collision_choice):
    # 用L1范数处理非线性运动学等式方程，车辆用一个圆覆盖
    # input: initialguess优化求解时需要一个初始猜测，此可由A*找到的路径提供；
    # solver_choice： 1-fmincon(matlab 自带，可处理任意函数)；2-cplexqp只能处理二次规划凸优化问题
    # collision_choice： 1-convex feasible set (CFS); 2-box;
    # kinematic_choice: 1-L1范数；2-L2范数
    # output:
    # trajectory.x,trajectory.y,trajectory.theta,trajectory.v,trajectory.phi
    # isFeasible: 如果求解器能求出可行解输出为1否则为0
    trajectory = trajectoryclass()
    # global planning_scale_ vehicle_geometrics_ vehicle_kinematics_ vehicle_TPBV_ obstacles_ Nobs margin_obs_
    isFeasible = 0
    Lm = vehicle_geometrics_.vehicle_wheelbase
    o_margin = margin_obs_  # 0.5
    nobj = Nobs
    obj = np.array([[]])
    if globalvar.Nobs > 0:
        lens = len(globalvar.obstacles_[0][0].x)
        obj = np.concatenate([(globalvar.obstacles_[0][0].x).reshape(
            1, lens), (globalvar.obstacles_[0][0].y).reshape(1, lens)], axis=0)
    for i in range(1, Nobs):
        obj = np.concatenate(
            [obj, (globalvar.obstacles_[0][i].x).reshape(1, lens), (globalvar.obstacles_[0][i].y).reshape(1, lens)], axis=0)
    # 对求解器赋初值
    nstep = len(initialguess.x)  # nstep=离散点的个数
    tf0 = np.zeros((1, 1))
    tf0[0][0] = nstep
    refpath_vec = np.zeros((2 * nstep, 1))
    refpath_vec[0::2] = np.array(initialguess.x).reshape((nstep, 1))
    refpath_vec[1::2] = np.array(initialguess.y).reshape((nstep, 1))
    theta0 = np.array(initialguess.theta).reshape((nstep, 1))
    theta = theta0 * 1
    v0 = np.array(initialguess.v).reshape((nstep, 1)) * 1
    vel = v0 * 1
    phi0 = np.array(initialguess.phi).reshape((nstep, 1)) * 1
    phi = phi0 * 1
    x0 = np.concatenate([refpath_vec, theta0, v0, phi0, tf0], axis=0)
    path_k = refpath_vec * 1
    Ts = x0[5 * nstep] / (nstep - 1)  # 采样时间delta_t

    # 决策变量数量
    dim = 2
    # 小车中心位置xn(2*nstep),theta(nstep),v(nstep),phi(nstep),tf(1)
    num_dv = nstep * 5 + 1
    # 辅助变量数量
    num_aux = 3 * nstep - 3
    # 避障约束最大数量
    num_collision_max = nstep * nobj

    # 算法参数设置
    # 回溯直线搜索的参数
    beta1 = 0.7
    alpha1 = 0.3
    step_size = 1
    # 优化算法内置参数
    iter = np.zeros(50)
    alpha = 2.5
    beta = 2.5
    radius0 = 4
    ratio = 3
    inner_maxiter = 50  # 最大内循环次数
    rho1 = 0.2
    rho2 = 0.9
    threshold_linear_error = 0.01
    inner_counter = 3  # 连续inner_counter次没有改进目标函数值，则退出循环
    # 停止阈值
    threshold = 1e-5
    # 外循环最大迭代次数
    maxiter = 5
    # 目标函数惩罚权重lambda
    lambda1 = 1e5  # 1e5
    # 回溯直线搜索merit function的权重
    mup = 1e3
    # 信任域参数设置
    radius_low = 1e-3  # 最小的信任域半径
    radius_up = 4.5  # 最大的信任域半径

    # 离散化的运动学方程
    Apdyn = np.zeros((nstep - 1, dim * nstep))
    Aqdyn = np.zeros((nstep - 1, dim * nstep))
    Athetadyn = np.zeros((nstep - 1, nstep))

    # 离散的代数状态方程
    Av1 = np.zeros((nstep - 1, num_dv + num_aux))
    Av2 = np.zeros((nstep - 1, num_dv + num_aux))
    Aphi1 = np.zeros((nstep - 1, num_dv + num_aux))
    Aphi2 = np.zeros((nstep - 1, num_dv + num_aux))

    # 新的
    Obj1 = np.zeros((nstep - 1, num_dv + num_aux))
    Obj2 = np.zeros((nstep - 1, num_dv + num_aux))

    for k in range(nstep - 1):
        # 目标函数
        Obj1[k][3 * nstep + k:3 * nstep + k + 2] = [-1, 1]
        Obj2[k][4 * nstep + k:4 * nstep + k + 2] = [-1, 1]
        # 离散化的运动学方程
        Apdyn[k][2 * k:2 * k + 3] = [-1, 0, 1]
        Aqdyn[k][2 * k + 1:2 * k + 4] = [-1, 0, 1]
        Athetadyn[k][k:k + 2] = [-1, 1]
        # 离散的代数状态方程
        # index1 = [3 * nstep + k, 3 * nstep + k + 1, num_dv]
        Av1[k][3 * nstep + k] = -1
        Av1[k][3 * nstep + k + 1] = 1
        Av1[k][num_dv - 1] = -vehicle_kinematics_.vehicle_a_max / (nstep - 1)
        Av2[k][3 * nstep + k] = 1
        Av2[k][3 * nstep + k + 1] = -1
        Av2[k][num_dv - 1] = -vehicle_kinematics_.vehicle_a_max / (nstep - 1)
        # index2 = [4 * nstep + k, 4 * nstep + k + 1, num_dv]
        Aphi1[k][4 * nstep + k] = -1
        Aphi1[k][4 * nstep + k + 1] = 1
        Aphi1[k][num_dv - 1] = - \
            vehicle_kinematics_.vehicle_omega_max / (nstep - 1)
        Aphi2[k][4 * nstep + k] = 1
        Aphi2[k][4 * nstep + k + 1] = -1
        Aphi2[k][num_dv - 1] = - \
            vehicle_kinematics_.vehicle_omega_max / (nstep - 1)

    k = nstep

    # 目标函数的一次项
    F = np.zeros((num_dv + num_aux, 1))
    F[num_dv - 1][0] = 1  # 使时间最小
    F[num_dv:] = lambda1 * np.ones((num_aux, 1))  # 惩罚等式约束
    f = np.zeros((num_dv, 1))
    f[num_dv - 1][0] = 1  # 使时间最小
    # 目标函数的二次项
    H = 2 * (np.dot(Obj1.T, Obj1) + np.dot(Obj2.T, Obj2))
    H_x = 2 * (np.dot(Obj1[:, 0:num_dv].T, Obj1[:, 0:num_dv]) +
               np.dot(Obj2[:, 0:num_dv].T, Obj2[:, 0:num_dv]))

    # 等式约束：限制轨迹的起止状态x,y,theta,v,phi
    Aeq0 = np.zeros((10, num_dv + num_aux))
    # [refpath_vec(1);refpath_vec(2);refpath_vec(nstep*dim-1);refpath_vec(nstep*dim);refpath_vec(3)-refpath_vec(1);refpath_vec(4)-refpath_vec(2);refpath_vec(end-1)-refpath_vec(end-3);refpath_vec(end)-refpath_vec(end-2)];
    beq0 = np.zeros((10, 1))
    Aeq0[0:2, 0:2] = np.eye(2)  # initial_x;initial_y
    Aeq0[2:4, nstep * 2 - 2:nstep * 2] = np.eye(2)  # end_x;end_y
    Aeq0[4][nstep * 2] = 1  # initial_theta
    Aeq0[5][nstep * 3 - 1] = 1  # end_theta
    Aeq0[6][nstep * 3] = 1  # initial_v
    Aeq0[7][nstep * 4 - 1] = 1  # end_v
    Aeq0[8][nstep * 4] = 1  # initial_phi
    Aeq0[9][nstep * 5 - 1] = 1  # end_phi
    beq0[0][0] = vehicle_TPBV_.x0  # initial_x
    beq0[1][0] = vehicle_TPBV_.y0  # initial_y
    beq0[2][0] = vehicle_TPBV_.xtf  # end_x
    beq0[3][0] = vehicle_TPBV_.ytf  # end_y
    beq0[4][0] = vehicle_TPBV_.theta0
    beq0[5][0] = vehicle_TPBV_.thetatf
    beq0[6][0] = vehicle_TPBV_.v0
    beq0[7][0] = vehicle_TPBV_.vtf
    beq0[8][0] = vehicle_TPBV_.phi0
    beq0[9][0] = vehicle_TPBV_.phitf

    # 边界条件约束
    lb = np.zeros((num_dv + num_aux, 1))
    ub = np.zeros((num_dv + num_aux, 1))
    # 轨迹横纵坐标的范围限制
    lb[0:dim * nstep:dim] = planning_scale_.xmin
    lb[1:dim * nstep:dim] = planning_scale_.ymin
    ub[0:dim * nstep:dim] = planning_scale_.xmax
    ub[1:dim * nstep:dim] = planning_scale_.ymax
    # theta的上下界（这里设置无穷大是为了防止在特殊场景下角度以2pi倍数增长）
    lb[2 * nstep:3 * nstep] = -10000
    ub[2 * nstep:3 * nstep] = 10000
    # v的上下界
    lb[3 * nstep:4 * nstep] = vehicle_kinematics_.vehicle_v_min
    ub[3 * nstep:4 * nstep] = vehicle_kinematics_.vehicle_v_max
    # phi的上下界
    lb[4 * nstep:5 * nstep] = vehicle_kinematics_.vehicle_phi_min
    ub[4 * nstep:5 * nstep] = vehicle_kinematics_.vehicle_phi_max
    # 行驶时间的上下界
    lb[5 * nstep][0] = 0
    ub[5 * nstep][0] = 1000
    # 等式惩罚
    lb[num_dv:] = 0  # lb(num_dv+1:end)=-inf;
    ub[num_dv:] = 10000
    # 信任域约束
    Tr = np.zeros((num_dv, num_dv + num_aux))
    Tr[0:num_dv, 0:num_dv] = np.eye(num_dv)

    # 计算x0的运动学约束满足情况
    F_x0 = np.concatenate([np.dot(Apdyn, x0[0:2 * nstep]) - np.multiply(Ts[0] * vel[0:nstep - 1], np.cos(theta[0:nstep - 1])),
                          np.dot(Aqdyn, x0[0:2 * nstep]) - np.multiply(Ts[0]
                                                                       * vel[0:nstep - 1], np.sin(theta[0:nstep - 1])),
                          np.dot(Athetadyn, theta[0:nstep]) - np.multiply(Ts[0] / Lm * vel[0:nstep - 1], np.tan(phi[0:nstep - 1]))], axis=0)
    # print(F_x0)
    # 对运动学约束F(x)=0进行线性化（一阶泰勒展开），得到delta_Jac和delta_b
    delta_Jac = ComputeJac(x0, Lm)
    delta_b = - F_x0 + np.dot(delta_Jac, x0)
    # 总目标函数值（包括原始目标和惩罚项）
    J_x0 = 1/2 * np.dot(np.dot(x0.T, H_x), x0) + \
        np.dot(f.T, x0) + lambda1 * norm(F_x0)

    x_star = x0 * 1  # x_star是当前最好的轨迹
    # z_trajectory = [x0;F_x0];  注意辅助变量大于等于0！！！！
    z0 = np.concatenate([x0, np.fabs(F_x0)], axis=0)
    z = z0 * 1
    cost_counter = 0  # 统计已经进行了多少次内循环
    no_improve_counter = 0  # 统计已经多少次没有改进原始目标函数值了
    # cost(cost_counter) = 1/2 * x0' * H_x * x0 + f' * x0;
    # violation(cost_counter) = norm(F_x0, 1);
    cost = []
    cost.append(1/2 * np.dot(np.dot(x0.T, H_x), x0) + np.dot(f.T, x0))
    violation = []
    violation.append(norm(F_x0))
    cost_violation = np.array([[]])  # 这个所有约束的违反情况

    eq_flag = 0
    # 开始迭代
    for k in range(maxiter):
        radius = radius0  # 每次外循环迭代当障碍物没有时，只是信任域半径变了，重新又增大了
        # 避障约束
        A = np.zeros((num_collision_max, num_dv + num_aux))
        b = np.zeros((num_collision_max, 1))
        # 避障约束
        counter_collision = 1  # 统计避障约束的数量
        # 对于每一个时刻进行迭代
        for i in range(1, nstep + 1):
            # indexi = (i - 1) * dim + 1:i * dim;
            xnr = path_k[(i - 1) * dim:i * dim] * 1  # xnr是列向量
            # 为xnr计算凸可行集
            if collision_choice == 1:
                tempA, tempb = Find_Box_for_all_obj(xnr)
            elif collision_choice == 2:
                tempA, tempb, _ = Find_CFS_for_all_obj(xnr, obj)

            num_tempA = np.size(tempb, 0)
            if num_tempA > 0:
                if(counter_collision - 1 + num_tempA > A.shape[0]):
                    A = np.concatenate([A, np.zeros(
                        (counter_collision - 1 + num_tempA - A.shape[0], num_dv + num_aux))], axis=0)
                    b = np.concatenate(
                        [b, np.zeros((counter_collision - 1 + num_tempA - b.shape[0], 1))], axis=0)

                A[counter_collision - 1: counter_collision -
                    1 + num_tempA, 2 * (i - 1): 2 * i] = tempA * 1

                if collision_choice == 1:
                    b[counter_collision - 1: counter_collision -
                        1 + num_tempA] = tempb * 1
                elif collision_choice == 2:
                    b[counter_collision - 1: counter_collision -
                        1 + num_tempA] = tempb - o_margin

                counter_collision = counter_collision + num_tempA
            # 计算其他圆心xir的凸可行集约束，并将其转换到决策变量xr和xnr的线性约束

        num_collision = counter_collision - 1
        num_constraint = num_collision + 4 * \
            (nstep - 1) + 2 * (5 * nstep - 3) + 2 * num_dv
        Aineq = np.zeros((num_constraint, num_dv + num_aux))
        bineq = np.zeros((num_constraint, 1))

        # 外循环是CFS的循环，内循环是针对固定的凸可行集进行优化的循环
        Aineq[:num_collision] = A[: num_collision] * 1
        Aineq[num_collision: num_collision + nstep - 1] = Av1 * 1
        Aineq[num_collision + nstep -
              1: num_collision + 2 * (nstep - 1)] = Av2 * 1
        Aineq[num_collision + 2 *
              (nstep - 1): num_collision + 3 * (nstep - 1)] = Aphi1 * 1
        Aineq[num_collision + 3 *
              (nstep - 1): num_collision + 4 * (nstep - 1)] = Aphi2 * 1
        Aineq[num_collision + 4 * nstep - 3 - 1: num_collision + 4 *
              nstep - 4 + num_aux, num_dv: num_dv + num_aux] = -np.eye(num_aux)
        Aineq[num_collision + 4 * nstep - 3 + num_aux - 1: num_collision + 4 *
              nstep - 4 + 2 * num_aux, num_dv: num_dv + num_aux] = -np.eye(num_aux)
        Aineq[num_collision + 4 * nstep - 4 + 2 * num_aux: num_collision +
              4 * nstep - 4 + 2 * num_aux + num_dv] = Tr * 1
        Aineq[num_collision + 4 * nstep - 4 + 2 * num_aux +
              num_dv: num_collision + 4 * nstep - 4 + 2 * num_aux + 2 * num_dv] = -Tr * 1

        bineq[: num_collision] = b[: num_collision] * 1

        z_last = z0 * 1  # 保存上一次的解

        for inner_index in range(1, inner_maxiter):
            Aineq[num_collision + 4 * nstep - 3 - 1:
                  num_collision + 4 * nstep - 4 + num_aux, : num_dv] = delta_Jac * 1
            Aineq[num_collision + 4 * nstep - 4 + num_aux:
                  num_collision + 4 * nstep - 4 + 2 * num_aux, : num_dv] = -delta_Jac * 1

            bineq[num_collision + 4 * nstep - 3 - 1:
                  num_collision + 4 * nstep - 4 + num_aux] = delta_b * 1
            bineq[num_collision + 4 * nstep - 4 + num_aux:
                  num_collision + 4 * nstep - 4 + 2 * num_aux] = - delta_b * 1  # -delta_b

            # print(delta_b)

            # x0+radius
            bineq[num_collision + 4 * nstep - 4 + 2 * num_aux:
                  num_collision + 4 * nstep - 4 + 2 * num_aux + num_dv] = x0 + radius
            # -x0+radius
            bineq[num_collision + 4 * nstep - 4 + 2 * num_aux + num_dv:
                  num_collision + 4 * nstep - 4 + 2 * num_aux + 2 * num_dv] = -x0 + radius

            cost_counter = cost_counter + 1

            # 在未计算前就统计约束违反情况
            ceq1 = np.dot(Aeq0, z0) - beq0
            theta = z0[2 * nstep: 3 * nstep] * 1
            vel = z0[3 * nstep: 4 * nstep] * 1
            phi = z0[4 * nstep: 5 * nstep] * 1
            Ts = z0[5 * nstep][0] / (nstep - 1)

            ceq2 = np.concatenate([np.dot(Apdyn, z0[:2 * nstep]) - np.multiply(np.dot(Ts, vel[:nstep - 1]), np.cos(theta[:nstep - 1])),
                                   np.dot(Aqdyn, z0[:2 * nstep]) - np.multiply(
                                       np.dot(Ts, vel[:nstep - 1]), np.sin(theta[:nstep - 1])),
                                   np.dot(Athetadyn, theta[:nstep]) - np.multiply(np.dot(Ts / Lm, vel[:nstep - 1]), np.tan(phi[:nstep - 1]))], axis=0)
            ceq = np.concatenate([ceq1, ceq2], axis=0)

            # 不等式
            cineq1 = np.dot(Aineq[: num_dv], z0) - bineq[: num_dv]
            cineq2 = lb[: num_dv] - z0[: num_dv]
            cineq3 = z0[: num_dv] - ub[: num_dv]
            cineq = np.concatenate([cineq1, cineq2, cineq3], axis=0)

            # 定义约束违反度
            vv = np.array(
                [[norm(np.where(cineq > 0, cineq, 0)), norm(ceq1), norm(ceq2)]])
            if(cost_violation.size == 0):
                cost_violation = 1 * vv
            else:
                cost_violation = np.concatenate([cost_violation, vv], axis=0)

            if norm(ceq2) < 0.001:  # 如果约束满足则直接退出循环  1e-2
                x = z0[: num_dv] * 1
                isFeasible = 1
                eq_flag = 1
                break

            # 如果前后两次约束违反的变化量小于一定值也退出
            if k > 0:
                if norm(cost_violation[k][2] - cost_violation[k - 1][2]) < 0.001:
                    break

            # 求解底层优化问题
            z, exitflag = quadprog(H, F, Aineq, bineq, Aeq0, beq0, lb, ub, z0)
            if (not (exitflag == 'optimal')) or (sum(np.isnan(z)) > 0) or (z.any() == False):
                isFeasible = 0
                x = z0[: num_dv] * 1
                trajectory.x = z_last[: dim * nstep: 2] * 1
                trajectory.y = z_last[1: dim * nstep: 2] * 1
                trajectory.theta = z_last[2 * nstep: 3 * nstep] * 1
                trajectory.v = z_last[3 * nstep: 4 * nstep] * 1
                trajectory.phi = z_last[4 * nstep: 5 * nstep] * 1
                trajectory.tf = z_last[5 * nstep] * 1
                break
            else:
                z0 = z * 1
                z_last = z * 1

            aux = z[num_dv:] * 1
            x = z[:num_dv] * 1

            # 计算delta_J和delta_L
            theta = x[2 * nstep: 3 * nstep] * 1
            vel = x[3 * nstep: 4 * nstep] * 1
            phi = x[4 * nstep: 5 * nstep] * 1
            Ts = x[5 * nstep] / (nstep - 1)
            F_x = np.concatenate([np.dot(Apdyn, x[0:2 * nstep]) - np.multiply(Ts * vel[:nstep - 1], np.cos(theta[:nstep - 1])),
                                  np.dot(Aqdyn, x[:2 * nstep]) - np.multiply(
                                      Ts * vel[:nstep - 1], np.sin(theta[:nstep - 1])),
                                  np.dot(Athetadyn, theta[:nstep]) - np.multiply(Ts / Lm * vel[:nstep - 1], np.tan(phi[:nstep - 1]))], axis=0)
            J_x = 1/2 * np.dot(np.dot(x.T, H_x), x) + \
                np.dot(f.T, x) + lambda1 * norm(F_x, ord=1)
            delta_J = J_x0 - J_x
            L_x = 1/2 * np.dot(np.dot(x.T, H_x), x) + \
                np.dot(f.T, x) + lambda1 * norm(aux, ord=1)
            delta_L = J_x0 - L_x

            if norm(F_x, ord=1) < 1e-5:  # 新增语句，如果约束满足则直接退出循环  1e-2
                isFeasible = 1
                eq_flag = 1
                break
            # print(delta_L)
            # 如果是内循环的第一次迭代，则直接接受
            if ((inner_index == 1) and (delta_L > threshold)):
                x0 = x * 1
                F_x0 = F_x * 1
                delta_Jac = ComputeJac(x0, Lm)
                delta_b = -F_x0 + np.dot(delta_Jac, x0)
                J_x0 = J_x * 1
                # print(delta_b)

                # z0 = [x0; abs(F_x0)]
                z0 = np.concatenate([x0, np.array[np.fabs(F_x0)]], axis=0)
                cost.append(1/2 * np.dot(np.dot(x0.T, H_x), x0) +
                            np.dot(f.T, x0))
                violation.append(norm(F_x0, ord=1))
                radius = radius / ratio  # 跳变时进行大范围搜索，因此使用大的信任域半径，局部优化使用小信任域半径
                continue

            if delta_J < -1e-3:  # 说明没有起到改进轨迹的作用（信任域较大时一般会出现这种情况）

                if radius <= radius_low:  # 注意：主要是在这一步退出程序的，改进很小！！
                    break
                while(len(violation) < cost_counter):
                    violation.append(0)
                if violation[cost_counter - 1] < 0.001:
                    no_improve_counter = no_improve_counter + 1

                    if no_improve_counter >= inner_counter:
                        break

                radius = max(radius / alpha, radius_low)
                # 回溯直线搜索
                delta_x = (x - x0)

                # 如果改变量很接近，则退出内循环
                if norm(delta_x) > 0.001 * nstep * (dim):
                    merit_function_x0 = 1/2 * \
                        np.dot(np.dot(x0.T, H_x), x0) + \
                        np.dot(f.T, x0) + mup * norm(F_x0, ord=1)
                    D_merit_function_x0 = np.dot(
                        (np.dot(x0.T, H_x) + f.T + mup * np.dot(np.sign(F_x0).T, delta_Jac)), delta_x)

                    for line_search in range(15):
                        temp_x = x0 + step_size * delta_x
                        linear_error = norm(temp_x - x0, np.inf)
                        F_tempx = np.concatenate([np.dot(Apdyn, temp_x[:2 * nstep]) - np.multiply((temp_x[5 * nstep] / (nstep - 1)) * temp_x[3 * nstep:4 * nstep - 1], np.cos(temp_x[2 * nstep:3 * nstep - 1])),
                                                  np.dot(Aqdyn, temp_x[:2 * nstep]) - np.multiply((temp_x[5 * nstep] / (
                                                      nstep - 1)) * temp_x[3 * nstep:4 * nstep - 1], np.sin(temp_x[2 * nstep:3 * nstep - 1])),
                                                  np.dot(Athetadyn, temp_x[2 * nstep:3 * nstep]) - np.multiply((temp_x[5 * nstep] / (nstep - 1)) / Lm * temp_x[2 * nstep:3 * nstep - 1], np.tan(temp_x[2 * nstep:3 * nstep - 1]))], axis=0)
                        merit_function_tempx = 1/2 * \
                            np.dot(np.dot(temp_x.T, H_x), temp_x) + \
                            np.dot(f.T, temp_x) + mup * norm(F_tempx, ord=1)
                        thershold = merit_function_x0 + alpha1 * step_size * D_merit_function_x0

                        if merit_function_tempx <= thershold:
                            x0 = temp_x * 1
                            F_x0 = F_tempx * 1
                            break
                        else:
                            step_size = step_size * beta1

                    radius = min(radius, linear_error)
                else:
                    x0 = x * 1
                    break

            elif math.fabs(delta_J) < threshold:  # delta_J<threshold
                break

            elif delta_J > 1:
                x0 = x * 1
                F_x0 = F_x * 1
                delta_Jac = ComputeJac(x0, Lm)
                delta_b = -F_x0 + np.dot(delta_Jac, x0)
                J_x0 = J_x * 1
                z0 = np.concatenate([x0, np.fabs(F_x0)],
                                    axis=0)  # z0 = [x0;F_x0];

                # 判断线性化近似程度，以调整信任域半径
                if norm(aux, ord=1) > threshold_linear_error:
                    rho_k = norm(aux, ord=1) / norm(F_x, ord=1)
                else:
                    rho_k = threshold_linear_error / norm(F_x, ord=1)

                if rho_k < rho1:  # 线性化误差较大，缩小信任域
                    radius = max(radius / alpha, radius_low)
                else:
                    threshold_linear_error = 0.1 * threshold_linear_error

                    if rho_k >= rho2:  # 线性化误差较小，可适当扩大信任域
                        radius = min(beta * radius, radius_up)

            cost.append(1/2 * np.dot(np.dot(x0.T, H_x), x0) + np.dot(f.T, x0))
            violation.append(norm(F_x0, ord=1))

            while(len(violation) <= cost_counter):
                violation.append(0)
            # 如果约束违反程度较小了，认为此时快进入收敛阶段了，则直接将信任域半径设置为较小的值
            if ((norm(aux, ord=1) < 1e-4) and ((violation[cost_counter] < 0.1)) or ((violation[cost_counter] < 0.5) and (math.fabs(violation[cost_counter] - violation[cost_counter - 1] < 0.1)))):
                radius = min(radius, 0.01)

        no_improve_counter = 0
        iter[k] = inner_index * 1

        if norm(F_x0) < 0.01:  # 1e-2 %1e-4
            eq_flag = 1

        if eq_flag == 1:  # 新增语句，如果约束满足则直接退出循环
            break

    # 在结束后再统计一次计算违反约束的情况
    vv = np.array([[norm(np.where(cineq > 0, cineq, 0)),
                  norm(ceq1), norm(F_x0)]])
    cost_violation = np.concatenate([cost_violation, vv], axis=0)

    # 判断迭代maxiter次找到的轨迹是否满足运动学约束和避障约束（是否真正找到可行解）
    if isFeasible:

        if sum(lb - z) <= 0 and sum(z - ub) <= 0 and sum(np.dot(Aineq, z) - bineq) <= 0 and sum(np.fabs(np.dot(Aeq0, z) - beq0)) <= 0.1:
            isFeasible = 1
        else:
            isFeasible = 0

    # 如果用求解器cplexqp迭代很多次仍未找到可行解，试用minimize求解器再试图求解
    # 详见L2文件
    

    x = z[: num_dv] * 1

    # 再次对求得的解进行约束检查
    if isFeasible == 1:

        if (norm(kinematiccon(x, nstep, Apdyn, Aqdyn, Athetadyn, Lm)) < 1e-3 and sum(lb[: num_dv] - x) <= 0
                and sum(x - ub[: num_dv]) <= 0 and sum(np.dot(Aineq[: num_collision + 4 * (nstep - 1), : num_dv], x) - bineq[: num_collision + 4 * (nstep - 1)]) <= 0
                and sum(np.fabs(np.dot(Aeq0[:, : num_dv], x) - beq0)) <= 0.1):
            isFeasible = 1
        else:
            isFeasible = 0

    trajectory.x = x[:dim * nstep:2] * 1
    trajectory.y = x[1:dim * nstep:2] * 1
    trajectory.theta = x[2 * nstep:3 * nstep] * 1
    trajectory.v = x[3 * nstep: 4 * nstep] * 1
    trajectory.phi = x[4 * nstep: 5 * nstep] * 1
    trajectory.tf = x[5 * nstep] * 1

    return trajectory, isFeasible
