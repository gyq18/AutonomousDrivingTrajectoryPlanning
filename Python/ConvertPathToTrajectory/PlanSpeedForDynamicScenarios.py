import numpy as np 
import heapq
from save_video import save_video
# from statelatticeplanner import CreateVehiclePolygon, inpolygon
from statelatticeplanner.main_unstructure import CreateVehiclePolygon, inpolygon
from types import SimpleNamespace
from cvxopt import solvers, matrix
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import io
import cv2
import seaborn as sns

def PlanSpeedForDynamicScenarios(x, y, theta, config):
    t, s = SearchVelocityInStGraph(x, y, theta, config)

    # s, t = OptimizeVelocityInStGraph(t, s, config)
    # plt.figure()
    # plt.plot(t0, s0)
    # plt.show()

    # plt.figure()
    # sns.heatmap(config.costmap_)
    # plt.show()
    trajectory = TransformPathToTrajectoryForDynamicScenarios(x, y, theta, t, s, config)
    # print(s0)
    # print(s)
    # print(t)
    # print(trajectory.x)

    return trajectory


def SearchVelocityInStGraph(x, y, theta, config):
    st_graph_search_ = config.st_graph_search_


    costmap_ = np.zeros((st_graph_search_.num_nodes_t, st_graph_search_.num_nodes_s))

    # why global var here?
    dynamic_obs = config.dynamic_obs
    nobs = np.asarray(dynamic_obs).shape[0]

    for i in range(st_graph_search_.num_nodes_t):
        for j in range(st_graph_search_.num_nodes_s):
            index = int(j * len(x) / st_graph_search_.num_nodes_s)
            current_x = x[index]
            current_y = y[index]
            current_theta = theta[index]

            for k in range(nobs):
                if IsVehicleCollidingWithMovingObstacle(current_x, current_y, current_theta, dynamic_obs[k][i], config):
                    costmap_[i, j] = 1
                    continue
    config.costmap_ = costmap_
    path =  SearchStPathViaAStar(config)

    path = np.asarray(path)
    t = path[:, 0]
    s = path[:, 1]

    return t, s



# def index2st(index):
#     global st_graph_search_
#     # x for t, y for s
#     t = int(index // st_graph_search_.num_nodes_s)
#     s = index - t * st_graph_search_.num_nodes_s

#     return (s, t)

# def st2index(s, t):
#     global st_graph_search_
#     return s + t * st_graph_search_.num_nodes_s


def SearchStPathViaAStar(config):
    st_graph_search_ = config.st_graph_search_
    num_s = st_graph_search_.num_nodes_s
    num_t = st_graph_search_.num_nodes_t
    costmap_ = config.costmap_
    
    grid_space_2d_ = []
    # (delta_t, delta_s)
    expansion_pattern = [
        [0, 1],
        [0, -1],
        [1, 1],
        [1, -1],
        [1, 0 ] ]
    expansion_length = [
        1 + st_graph_search_.penalty_for_inf_velocity,
        1 + st_graph_search_.penalty_for_inf_velocity,
        1.414,
        1.414,
        1]

    g_score = np.full((num_t, num_s), np.inf)
    start_ts = (0, 0)
    end_ts = (num_t - 1, num_s - 1)
    g_score[start_ts] = 0
    h_start = abs(end_ts[0] - start_ts[0]) +  abs(end_ts[1] - start_ts[1])

    open_list = []
    close_list = []

    heapq.heappush(open_list, (h_start, start_ts))
    path = {}

    while len(open_list) != 0:
        current_f, current_ts = heapq.heappop(open_list)
        close_list.append(current_ts)

        if current_ts == end_ts:
            break

        for move, cost in zip(expansion_pattern, expansion_length):

            next_ts = (current_ts[0] + move[0], current_ts[1] + move[1])

            # judge if st is valid
            if not (0 <= next_ts[0] < num_t and 0 <= next_ts[1] < num_s):
                continue

            if next_ts in close_list:
                continue

            temp_g_score = g_score[current_ts] + cost

            if temp_g_score < g_score[next_ts]:
                # check if it's collision free
                if costmap_[next_ts] == 1:
                    continue

                # calculate f score
                g_score[next_ts] = temp_g_score
                h_score = abs(end_ts[0] - next_ts[0]) +  abs(end_ts[1] - next_ts[1]) * st_graph_search_.multiplier_H_for_A_star
                f_score = g_score[next_ts] + h_score
                
                # push new node to open list
                heapq.heappush(open_list, (f_score, next_ts))
                # save path
                path[next_ts] = current_ts


    # min_g_ts = None
    # if not np.any(g_score[:, -1] < np.inf): 
    #     return []
    # else:
    #     min_g_ts = (np.argmin(g_score[:, -1]), end_ts[1])
    if end_ts not in path:
        return []

    path_result = []
    path_result.append(end_ts)
    current_ts = end_ts
    while current_ts in path:
        path_result.append(path[current_ts])
        current_ts = path[current_ts]
    path_result.reverse()
    
    return path_result

def IsVehicleCollidingWithMovingObstacle(x, y, theta, V, config):
    if np.min(np.hypot(V.x-x, V.y-y)) > 10:
        return 0
    Vcar = CreateVehiclePolygonFull(x, y, theta, config)
    for xv, yv  in zip(Vcar.x, Vcar.y):
        if inpolygon(xv, yv , V.x, V.y) :
            return 1
    
    return 0
    


def CreateVehiclePolygonFull(x, y, theta, config):
    vehicle_geometrics_ = config.vehicle_geometrics_
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # Here we use a SimplenNameSpace as an anonymous object
    Vcar = SimpleNamespace()
    vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5
    AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta
    BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta
    CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta
    DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta
    AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta
    BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta
    CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta
    DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta
    Vcar.x = [AX, BX, CX, DX, AX]
    Vcar.y = [AY, BY, CY, DY, AY]

    return Vcar


def OptimizeVelocityInStGraph(t0,s0, config):
    st_graph_search_ = config.st_graph_search_
    vehicle_kinematics_ = config.vehicle_kinematics_

    delta_t = st_graph_search_.resolution_t
    t = t0
    nstep = len(s0)
    s0 = np.array(s0).T

    Q1 = np.eye(nstep)

    Vdiff = (np.eye(nstep) - np.diag(np.ones((1, nstep-1)), 1)) * delta_t

    Q2 = Vdiff[:(nstep-1), :].T @ Q1[1:, 1:] @ Vdiff[:(nstep-1), :]

    Adiff = np.eye(nstep) - 2 * np.diag(np.ones((1, nstep - 1)), 1) + np.diag(np.ones((1, nstep-2)), 2)
    # is that right in dim?
    Adiff = Adiff * delta_t ** 2
    Q3 = Adiff[:(nstep-2), :].T @ Adiff[:(nstep-2), :]

    Jdiff = np.eye(nstep) - 3 * np.diag(np.ones((1, nstep - 1)), 1) + 3 * np.diag(np.ones((1, nstep - 2)), 2) - np.diag(np.ones((1, nstep - 3)), 3)
    Jdiff = Jdiff * delta_t ** 3
    Q4 = Jdiff[:(nstep-2), :].T @ Jdiff[:(nstep-2), :]

    cref = [1, 0, 0, 0]
    cabs = [0, 1, 1, 1]

    Qref = Q1 * cref[0] + Q2 * cref[1] + Q3 * cref[2] + Q4 * cref[3]
    Qabs = Q1 * cabs[0] + Q2 * cabs[1] + Q3 * cabs[2] + Q4 * cabs[3]

    Aeq = np.zeros((2, nstep))
    Aeq[0, 0] = 1
    Aeq[1, -1] = 1
    beq = np.array([[s0[0]], [s0[-1]]])

    lb = np.zeros((nstep, 1))
    ub = max(s0) * np.ones((nstep, 1))

    Aineq = np.vstack([Vdiff[:(nstep-1), :], -Vdiff[:(nstep-1), :], np.eye(nstep), -np.eye(nstep)])
    v_max = vehicle_kinematics_.vehicle_v_max
    v_min = vehicle_kinematics_.vehicle_v_min
    bineq = np.vstack([v_max * np.ones((nstep-1, 1)), -v_min * np.ones((nstep-1, 1)), ub, -lb])

    H = Qref + Qabs
    f = - Qref @ s0
    # np.set_printoptions(threshold=np.inf)
    # fi = open("data.txt", "w")

    # print(H, file=fi)
    # print(f, file=fi)

    # TODO: what should be done to lb and ub??
    s = solvers.qp(matrix(H), matrix(f), matrix(Aineq), matrix(bineq), matrix(Aeq), matrix(beq, tc='d'), initvals=np.linspace(s0[0], s0[-1], len(s0)))

    return np.array(s['x'])[:, 0], t
    

def TransformPathToTrajectoryForDynamicScenarios(x, y, theta, t, s, config):
    st_graph_search_ = config.st_graph_search_
    vehicle_geometrics_ = config.vehicle_geometrics_
    vehicle_kinematics_ = config.vehicle_kinematics_

    delta_t = st_graph_search_.resolution_t
    # delete redundant valus
    if len(t) > st_graph_search_.num_nodes_t:
        # confusing naming
        tt = [t[-1]]
        ss = [s[-1]]

        for i in range(len(t)-2, -1, -1):
            if t[i+1] != t[i]:
                tt.append(t[i])
                ss.append(s[i])
        tt.reverse()
        ss.reverse()
        tt[0] = 0
        ss[0] = 0
        t = tt
        s = ss

    assert len(t) == st_graph_search_.num_nodes_t, "Error"

    x, y, theta = ResamplePathWithEqualDistance( x, y, theta )

    # use ss again? 
    ss = np.zeros((len(x),))
    for i in range(1, len(x)):
        ss[i] = ss[i-1] + np.hypot(x[i] - x[i-1], y[i] - y[i-1])
    
    ss = ss / np.max(ss) * np.max(s)
    trajectory = SimpleNamespace()
    trajectory.x = []
    trajectory.y = []
    trajectory.theta = []

    for i in range(len(s)):
        # It's a matrix
        # I can't understand what's happening here
        err = np.abs(s[i]-ss)
        ind_min = np.argmin(err)

        # ind_min = int(ind_min * len(ss)/len(s))

        trajectory.x.append(x[ind_min])
        trajectory.y.append(y[ind_min])
        trajectory.theta.append(theta[ind_min])
    # trajectory.x.reverse()
    # trajectory.y.reverse()
    # trajectory.theta.reverse()

    Nfe = len(trajectory.x)
    vdr = np.zeros((Nfe,))

    # Judge velocity direction
    for i in range(1, Nfe-1):
        addition = (x[i+1] - x[i]) * np.cos(theta[i]) + (y[i+1] - y[i]) * np.sin(theta[i])
        if addition > 0:
            vdr[i] = 1
        else:
            vdr[i] = -1

    v = np.zeros((Nfe,))
    a = np.zeros((Nfe,))
    dt = delta_t 
    for i in range(1, Nfe):
        v[i] = vdr[i] * np.sqrt(((x[i] - x[i-1]) / dt)**2 + ((y[i] - y[i-1]) / dt)**2)

    for i in range(1, Nfe):
        a[i] = (v[i] - v[i-1])/dt

    phi = np.zeros((Nfe,))
    omega = np.zeros((Nfe,))
    phi_max = vehicle_kinematics_.vehicle_phi_max
    omega_max = vehicle_kinematics_.vehicle_omega_max
    for i in range(1, Nfe-1):
        phi[i] = np.arctan((theta[i+1]-theta[i]) * vehicle_geometrics_.vehicle_wheelbase / (dt * v[i]))
        if phi[i] > phi_max:
            phi[i] = phi_max
        elif phi[i] < -phi_max:
            phi[i] = -phi_max
    
    for i in range(1, Nfe-1):
        omega[i] = (phi[i+1] - phi[i]) / dt
        if (omega[i] > omega_max):
            omega[i] = omega_max
        elif (omega[i] < -omega_max):
            omega[i] = -omega_max
    trajectory.v = v
    trajectory.a = a
    trajectory.phi = phi
    trajectory.omega = omega

    return trajectory

def ResamplePathWithEqualDistance( x, y, theta ):
    for i in range(len(theta)):
        while (theta[i] - theta[i-1]) > np.pi:
            theta[i] -= 2 * np.pi
        while (theta[i] - theta[i-1]) < -np.pi:
            theta[i] += 2 * np.pi

    x_extended = []
    y_extended = []
    theta_extended = []
    for i in range(len(theta)-1):
        distance = np.hypot(x[i+1]-x[i], y[i+1]-y[i])
        # It's seems weird
        LARGE_NUM = np.round(distance * 100).astype(int)
        # LARGE_NUM = 100

        temp = np.linspace(x[i], x[i+1], LARGE_NUM)
        x_extended.extend(temp[:-1])

        temp = np.linspace(y[i], y[i+1], LARGE_NUM)
        y_extended.extend(temp[:-1])

        temp = np.linspace(theta[i], theta[i+1], LARGE_NUM)
        theta_extended.extend(temp[:-1])

    x_extended.append(x[-1])
    y_extended.append(y[-1])
    theta_extended.append(theta[-1])
    index = np.round(np.linspace(0, len(x_extended)-1, 1000)).astype(int)
    x_res = np.asarray(x_extended)[index]
    y_res = np.asarray(y_extended)[index]
    theta_res = np.asarray(theta_extended)[index]

    return x_res, y_res, theta_res

    

def VisualizeDynamicResultsForDynamicScenarios(trajectory, config):
    x = trajectory.x
    y = trajectory.y
    theta = trajectory.theta
    vehicle_TPBV_ = config.vehicle_TPBV_
    obstacles_  = config.obstacles_
    Nobs  = config.Nobs
    dynamic_obs = config.dynamic_obs
    planning_scale_ = config.planning_scale_

    colorpool = np.array([ [237, 28, 36], [0, 162, 232], [34, 177, 76], [255, 127, 39] ]) / 255
    number_of_frame = len(x)



    x0 = vehicle_TPBV_.x0
    y0 = vehicle_TPBV_.y0
    theta0 = vehicle_TPBV_.theta0
    xtf = vehicle_TPBV_.xtf
    ytf = vehicle_TPBV_.ytf
    thetatf = vehicle_TPBV_.thetatf

    image_array = []
    for i in range(number_of_frame):
        fig = plt.figure(0)
        fig.clear()
        ax = fig.add_subplot(111)
        canvas = FigureCanvas(fig)

        ax.set_xlim([planning_scale_.xmin, planning_scale_.xmax])
        ax.set_ylim([planning_scale_.ymin, planning_scale_.ymax])
        # TODO: length angle and width is not set
        ax.arrow(x0, y0, np.cos(theta0), np.sin(y0))
        ax.arrow(xtf, ytf, np.cos(thetatf), np.sin(ytf))

        if Nobs > 0:
            for obs in obstacles_:
                # TODO: Obs format
                vertex_x = obs.x
                vertex_y = obs.y
                plt.fill(vertex_x, vertex_y, color=(0.7451,0.7451,0.7451))
        
        # plot dynamic obstacles
        for dyn_obs in dynamic_obs:
            obs_of_time = dyn_obs[i]
            plt.fill(obs_of_time.x, obs_of_time.y, color=(0.7451,0.7451,0.7451))

        x_current = x[i]
        y_current = y[i]
        theta_current = theta[i]

        V = CreateVehiclePolygon(x_current, y_current, theta_current, config)
        ax.plot(V.x, V.y, linewidth=1)

        ax.plot(x[:i], y[:i], 'b')
        canvas.draw()

        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        buf.seek(0)
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        img = cv2.imdecode(img_arr, 1)
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        image_array.append(image)
        # plt.show()

    save_video(image_array, "video.mp4", frame_speed=24)
    
        
