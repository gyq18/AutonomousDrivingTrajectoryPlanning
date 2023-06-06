import globalvar
import numpy as np
from numpy.linalg import norm as norm


class trajectoryclass:
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.theta = None
        self.v = None
        self.a = None
        self.phi = None
        self.omega = None


def TransformPathToTrajectory(x=None, y=None, theta=None, path_length=None, method_flag=None):
    ## method_flag:1-FulfillProfiles(x, y, theta);2-GYQProfiles
    trajectory = trajectoryclass()
    trajectory.x = x
    trajectory.y = y
    if method_flag == 1:
        trajectory.theta, trajectory.v, trajectory.a, trajectory.phi, trajectory.omega = FulfillProfiles(x, y, theta,
                                                                                                         path_length)
    elif method_flag == 2:
        trajectory.theta, trajectory.v, trajectory.a, trajectory.phi, trajectory.omega = GYQProfiles(x, y, theta)

    return trajectory


def FulfillProfiles(x=None, y=None, theta=None, path_length=None):
    Nfe = len(x)
    # Judge velocity direction
    vdr = np.zeros(Nfe)
    for ii in range(1, Nfe - 1):
        addtion = (x[ii + 1] - x[ii]) * np.cos(theta[ii]) + (y[ii + 1] - y[ii]) * np.sin(theta[ii])
        if (addtion > 0):
            vdr[ii] = 1
        else:
            vdr[ii] = - 1

    v = np.zeros(Nfe)
    a = np.zeros(Nfe)
    dt = path_length / Nfe
    for ii in range(1, Nfe):
        v[ii] = vdr[ii] * np.sqrt(((x[ii] - x[ii - 1]) / dt) ** 2 + ((y[ii] - y[ii - 1]) / dt) ** 2)

    for ii in range(1, Nfe):
        a[ii] = (v[ii] - v[ii - 1]) / dt

    phi = np.zeros(Nfe)
    omega = np.zeros(Nfe)

    vehicle_kinematics_ = globalvar.vehicle_kinematics_
    vehicle_geometrics_ = globalvar.vehicle_geometrics_
    phi_max = vehicle_kinematics_.vehicle_phi_max
    omega_max = vehicle_kinematics_.vehicle_omega_max
    for ii in range(1, Nfe - 1):
        phi[ii] = np.arctan((theta[ii + 1] - theta[ii]) * vehicle_geometrics_.vehicle_wheelbase / (dt * v[ii]))
        if (phi[ii] > phi_max):
            phi[ii] = phi_max
        else:
            if (phi[ii] < - phi_max):
                phi[ii] = - phi_max

    for ii in range(1, Nfe - 1):
        omega[ii] = (phi[ii + 1] - phi[ii]) / dt
        if (omega[ii] > omega_max):
            omega[ii] = omega_max
        else:
            if (omega[ii] < - omega_max):
                omega[ii] = - omega_max
    # print(theta,v,a,phi,omega)
    return theta, v, a, phi, omega


def GYQProfiles(x=None, y=None, theta=None):
    pass


def CalculateLength(path_x=None, path_y=None):
    tlength = 0
    max_dlength = 0

    nstep = len(path_x)
    for i in range(0, nstep - 1):
        dlength = norm(np.array([path_x[i + 1], path_y[i + 1]]) - np.array([path_x[i], path_y[i]]))
        tlength = tlength + dlength
        if max_dlength < dlength:
            max_dlength = dlength

    return tlength, max_dlength
