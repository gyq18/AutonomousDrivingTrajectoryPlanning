import cvxopt.solvers
import numpy as np
import scipy
from cvxopt import matrix
from scipy.special import comb
import time

import globalvar
from PlanPRMPath import PRM, CreateCostmap
from TransformPathToTrajectory import TransformPathToTrajectory
from corridor import inflate_box
from main_unstructure import VisualizeStaticResults, GenerateStaticObstacles_unstructured, VisualizeDynamicResults

import csv

class bezier(object):
    def __init__(self, time_list, corridor):
        self.corridor = corridor
        self.times = time_list
        self.segment = len(time_list)
        self.M = self.get_M()
        self.Q = self.get_Q()

    def Bernstein_base(self, i, t_i, time_seg, traj_order=6):
        C_i = comb(traj_order, i)
        return time_seg * C_i * pow(t_i, i) * pow(1 - t_i, traj_order - i)

    def get_Q(self):
        M = self.M

        for k in range(self.segment):
            Q_k = np.zeros((7, 7))
            for i in range(7):
                for j in range(7):
                    if i <= 3 or j <= 3:
                        continue
                    Q_k[i, j] = i * (i - 1) * (i - 2) * (i - 3) * j * (
                            j - 1) * (j - 2) * (j - 3) / (i + j - 7) * pow(
                        self.times[k], i + j - 7)
            if k == 0:
                Q = Q_k
                M = M * self.times[0]
                M_k = M
            else:
                Q = scipy.linalg.block_diag(Q, Q_k)
                M_k = M_k * self.times[k]
                M = scipy.linalg.block_diag(M, M_k)
        Q_0 = np.dot(np.dot(M.T, Q), M)
        # Q_0 = near_psd(Q_0)
        # Q_0 = Q_0.astype(np.double)
        return Q_0

    def get_Aeq_and_Beq(self, is_y, traj_order=6):

        n_all_poly = self.segment * (traj_order + 1)
        #####################################################
        # STEP2.1 p, v, a, j constraint in start
        Aeq_start = np.zeros((3, n_all_poly))  # Ascending order
        Aeq_start[0, 0:1] = 1 * self.times[0]  # c0
        Aeq_start[1,
        0:2] = traj_order * np.array([-1, 1])  # c'0 = n*(-c0 + c0)
        Aeq_start[2, 0:3] = traj_order * (traj_order - 1) * np.array([
            1, -2, 1
        ]) / self.times[0]  # c''0 = n * (n - 1) * (c2 - 2 * c1 + c0)
        beq_start = np.zeros((3, 1))
        if not is_y:
            beq_start[0, 0] = globalvar.vehicle_TPBV_.x0
        else:
            beq_start[0, 0] = globalvar.vehicle_TPBV_.y0

        #  #####################################################
        # STEP 2.2 p, v, a, j constraint in end
        Aeq_end = np.zeros((3, n_all_poly))  # Descending order
        Aeq_end[0, -1:] = 1 * self.times[-1]  # cn
        Aeq_end[1,
        -2:] = traj_order * np.array([-1, 1])  # c'n-1 = n*(cn -cn-1)
        Aeq_end[2, -3:] = traj_order * (traj_order - 1) * np.array(
            [1, -2, 1]) / self.times[
                              -1]  # c''n - 2 = n ^ 2 * (n - 1) * (cn - 2 * cn - 1 + cn - 2)

        beq_end = np.zeros((3, 1))
        if not is_y:
            beq_end[0, 0] = globalvar.vehicle_TPBV_.xtf
        else:
            beq_end[0, 0] = globalvar.vehicle_TPBV_.ytf
        #  #####################################################
        # STEP 2.3 position continuity constrain between 2 segments
        Aeq_con_p = np.zeros((self.segment - 1, n_all_poly))
        for k in range(self.segment - 1):
            Aeq_con_p[k, (k + 1) * (traj_order + 1) - 1] = 1 * self.times[k]
            Aeq_con_p[k, (k + 1) * (traj_order + 1)] = -1 * self.times[k + 1]
        beq_con_p = np.zeros((self.segment - 1, 1))
        #  ####################################################
        # STEP 2.4 velocity continuity constrain between 2 segments
        Aeq_con_v = np.zeros((self.segment - 1, n_all_poly))
        for k in range(
                self.segment -
                1):  # (c(n)) - c(n - 1)) segment 1 + (-c1 + c0) segment 2
            Aeq_con_v[k, (k + 1) * (traj_order + 1) - 2:(k + 1) *
                                                        (traj_order + 1)] = np.array([-1, 1])
            Aeq_con_v[k, (k + 1) * (traj_order + 1):(k + 1) *
                                                    (traj_order + 1) + 2] = np.array([1, -1])
        beq_con_v = np.zeros((self.segment - 1, 1))

        #  #####################################################
        # STEP 2.5 acceleration continuity constrain between 2 segments
        Aeq_con_a = np.zeros((self.segment - 1, n_all_poly))
        for k in range(
                self.segment - 1
        ):  # (c(n)) - 2 * c(n - 1) + c(n - 2)) segment 1 + (-c2 + 2 * c1 - c0) segment 2
            Aeq_con_a[k, (k + 1) * (traj_order + 1) - 3: (k + 1) * (traj_order + 1)] = np.array([1, -2, 1]) / \
                                                                                       self.times[k]
            Aeq_con_a[k, (k + 1) * (traj_order + 1): (k + 1) * (traj_order + 1) + 3] = np.array([-1, 2, -1]) / \
                                                                                       self.times[k + 1]
        beq_con_a = np.zeros((self.segment - 1, 1))

        #  #####################################################
        # STEP 2.6 jerk continuity constrain between 2 segments
        Aeq_con_j = np.zeros((self.segment - 1, n_all_poly))
        for k in range(
                self.segment - 1
        ):  # (c(n)) - 3 * c(n - 1) + 3 * c(n - 2) - c(n - 3)) segment 1 + (-c3 + 3 * c2 - 3 * c1 + c0) segment 2
            Aeq_con_j[k, (k + 1) * (traj_order + 1) - 4:(k + 1) *
                                                        (traj_order + 1)] = np.array([1, -3, 3, -1]) / pow(
                self.times[k], 2)
            Aeq_con_j[k, (k + 1) * (traj_order + 1):(k + 1) *
                                                    (traj_order + 1) + 4] = np.array([-1, 3, -3, 1]) / pow(
                self.times[k + 1], 2)

        beq_con_j = np.zeros((self.segment - 1, 1))
        #  #####################################################
        # combine all components to form Aeq and beq
        Aeq_con = np.vstack((Aeq_con_p, Aeq_con_v, Aeq_con_a, Aeq_con_j))
        beq_con = np.vstack((beq_con_p, beq_con_v, beq_con_a, beq_con_j))
        Aeq = np.vstack((Aeq_start, Aeq_end, Aeq_con))
        beq = np.vstack((beq_start, beq_end, beq_con))
        return Aeq, beq

    def get_Aieq_and_Bieq(self, is_y, traj_order=6):
        n_all_poly = self.segment * (traj_order + 1)
        #  #####################################################
        # STEP 3.2.1 p constraint
        coeff_p = np.ones((n_all_poly, 1))
        bieq_p = np.ones((2 * n_all_poly, 1))
        for k in range(self.segment):  # max
            coeff_p[k * (traj_order + 1):(k + 1) * (traj_order + 1),
            0] = self.times[k]
            if not is_y:
                bieq_p[k * (traj_order + 1):(k + 1) * (traj_order + 1),
                0] = self.corridor[k][1]
            else:
                bieq_p[k * (traj_order + 1):(k + 1) * (traj_order + 1),
                0] = self.corridor[k][3]
        for k in range(self.segment):  # -min
            if not is_y:
                bieq_p[(n_all_poly + k * (traj_order + 1)): (n_all_poly + (k + 1) * (traj_order + 1)), 0] = - \
                    self.corridor[k][0]
            else:
                bieq_p[(n_all_poly + k * (traj_order + 1)): (n_all_poly + (k + 1) * (traj_order + 1)), 0] = - \
                    self.corridor[k][2]

        Aieq_p = np.diag(coeff_p.ravel())

        Aieq_p = np.vstack((Aieq_p, -Aieq_p))

        #  #####################################################
        # STEP 3.2.2 v constraint
        n_ctr = traj_order  # the number of control posints after first deferention: traj_order
        n_eq = n_ctr * self.segment * 2  # number of equations(max and min constraints)
        Aieq_v = np.zeros((n_ctr * self.segment, n_all_poly))

        for k in range(self.segment):
            for n in range(n_ctr):
                index_col = k * (traj_order + 1) + n
                index_row = n + k * n_ctr
                Aieq_v[index_row, index_col:index_col +
                                            2] = traj_order * np.array([-1, 1])

        Aieq_v = np.vstack((Aieq_v, -Aieq_v))
        # bieq_v = ones(n_eq, 1) * v_max
        bieq_v_max = np.ones((n_ctr * self.segment,
                              1)) * globalvar.vehicle_kinematics_.vehicle_v_max
        bieq_v_min = np.ones((n_ctr * self.segment,
                              1)) * globalvar.vehicle_kinematics_.vehicle_v_min
        bieq_v = np.vstack((bieq_v_max, -bieq_v_min))

        #  #####################################################
        # STEP 3.2.3 a constraint
        n_ctr = traj_order - 1  # the number of control points after second differention: traj_order - 1
        n_eq = n_ctr * self.segment * 2  # number of equations(max and min constraints)
        Aieq_a = np.zeros((n_ctr * self.segment, n_all_poly))

        for k in range(self.segment):
            for n in range(n_ctr):
                index_col = k * (traj_order + 1) + n
                index_row = n + k * n_ctr
                Aieq_a[index_row, index_col: index_col + 3] = traj_order * (traj_order - 1) * np.array([1, -2, 1]) / \
                                                              self.times[k]
        Aieq_a = np.vstack((Aieq_a, -Aieq_a))
        bieq_a_max = np.ones((n_ctr * self.segment,
                              1)) * globalvar.vehicle_kinematics_.vehicle_a_max
        bieq_a_min = np.ones((n_ctr * self.segment,
                              1)) * globalvar.vehicle_kinematics_.vehicle_a_min
        bieq_a = np.vstack((bieq_a_max, -bieq_a_min))

        # STEP 3.2.4 j constraint
        n_ctr = traj_order - 2  # the number of control points after third differention: traj_order - 2
        n_eq = n_ctr * self.segment * 2  # number of equations(max and min constraints)
        Aieq_j = np.zeros((n_ctr * self.segment, n_all_poly))
        for k in range(self.segment):
            for n in range(n_ctr):
                index_col = k * (traj_order + 1) + n
                index_row = n + k * n_ctr
                Aieq_j[index_row, index_col:index_col + 4] = traj_order * (
                        traj_order - 1) * (traj_order - 2) * np.array(
                    [1, -3, 3, -1]) / pow(self.times[k], 2)

        Aieq_j = np.vstack((Aieq_j, -Aieq_j))
        bieq_j_max = np.ones(
            (n_ctr * self.segment,
             1)) * globalvar.vehicle_kinematics_.vehicle_jerk_max
        bieq_j_min = np.ones(
            (n_ctr * self.segment,
             1)) * globalvar.vehicle_kinematics_.vehicle_jerk_min
        bieq_j = np.vstack((bieq_j_max, -bieq_j_min))

        #  #####################################################
        # combine all components to form Aieq and bieq

        Aieq = np.vstack((Aieq_p, Aieq_v, Aieq_a, Aieq_j))
        bieq = np.vstack((bieq_p, bieq_v, bieq_a, bieq_j))
        return Aieq, bieq

    def get_M(self):
        M = np.array([[1, 0, 0, 0, 0, 0, 0], [-6, 6, 0, 0, 0, 0, 0],
                      [15, -30, 15, 0, 0, 0, 0], [-20, 60, -60, 20, 0, 0, 0],
                      [15, -60, 90, -60, 15, 0,
                       0], [-6, 30, -60, 60, -30, 6, 0],
                      [1, -6, 15, -20, 15, -6, 1]])
        return M


class SolveQP(bezier):
    def __init__(self, time_list, box_list):
        super().__init__(time_list, box_list)
        P = self.Q
        q = np.zeros((self.segment * 7, 1)).ravel()
        G_x, h_x = self.get_Aieq_and_Bieq(is_y=0)
        A_x, b_x = self.get_Aeq_and_Beq(is_y=0)
        G_y, h_y = self.get_Aieq_and_Bieq(is_y=1)
        A_y, b_y = self.get_Aeq_and_Beq(is_y=1)
        from nearestPD import nearestPD

        # from scipy.io import savemat
        # savemat('QPsolver.mat',
        #         {'P': P, 'q': q, 'G_x': G_x, 'h_x': h_x, 'G_y': G_y, 'h_y': h_y, 'A_x': A_x, 'b_x': b_x, 'A_y': A_y,
        #          'b_y': b_y})

        P = nearestPD(P / np.linalg.norm(P))
        cvxopt.solvers.options['maxiters'] = 1000
        cvxopt.solvers.options['show_progress'] = False
        sol_x = cvxopt.solvers.qp(P=matrix(P), q=matrix(q), G=matrix(G_x), h=matrix(h_x), A=matrix(A_x), b=matrix(b_x))[
            'x']
        sol_y = cvxopt.solvers.qp(P=matrix(P), q=matrix(q), G=matrix(G_y), h=matrix(h_y), A=matrix(A_y), b=matrix(b_y))[
            'x']
        # print(sol_x, sol_y)
        
        self.polycoeff_x = sol_x
        self.polycoeff_y = sol_y

    def get_Bezier_Pos(self):
        path = []
        for j in range(self.segment):
            T = np.linspace(0, 1, 11)
            for ind,t in enumerate(T):
                if j != self.segment-1 and ind==T.shape[0]-1:
                    continue
                x_pos = 0
                y_pos = 0
                traj_order = 6
                for i in range(traj_order + 1):
                    start_idx = j * (traj_order + 1)
                    basis_p = comb(traj_order, i) * pow(t, i) * pow(
                        (1 - t), traj_order - i)
                    x_pos += self.polycoeff_x[start_idx +
                                              i] * basis_p * self.times[j]
                    y_pos += self.polycoeff_y[start_idx +
                                              i] * basis_p * self.times[j]
                path.append(np.array([x_pos, y_pos]))
        path = np.array(path)
        x = path[:, 0]
        y = path[:, 1]
        theta = []
        for i in range(len(path) - 1):
            theta.append(np.arctan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0]))
        theta.append(0)
        theta = np.array(theta)
        return x, y, theta


if __name__ == "__main__":
    globalvar.obstacles_ = GenerateStaticObstacles_unstructured()
    globalvar.costmap_ = CreateCostmap()
    t1 = time.time()
    [x, y, theta, path_length, completeness_flag] = PRM()
    t2 = time.time()
    print("PRM time: ", round(t2-t1,3))
    transmethod_flag = 1  # choose different path to trajectory method
    trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag)

    path = np.vstack((x, y)).T
    t3 = time.time()
    bezier_args = inflate_box(path)
    #VisualizeStaticResults(trajectory, bezier_args.vis_list, bezier_args.box_list, bezier_args.pt_list)

    bezier_path = SolveQP(bezier_args.time_allocated, bezier_args.corridor)
    x, y, theta = bezier_path.get_Bezier_Pos()
    t4 = time.time()
    print("Bezier time: ", round(t4-t3,3))
    time_measure = [round(t2-t1,3),round(t4-t3,3)]
    
    # with open('time.csv', 'a+', newline='') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(time_measure)

    trajectory = TransformPathToTrajectory(x, y, theta, path_length, transmethod_flag)

    VisualizeStaticResults(trajectory)
    VisualizeDynamicResults(trajectory)
