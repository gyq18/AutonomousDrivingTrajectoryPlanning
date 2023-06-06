import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.colors as colors
import matplotlib.cm as cmx

import sys
sys.path.append(".")


class Task_:
    def __init__(self) -> None:
        self.x0 = []
        self.y0 = []
        self.theta0 = []
        self.xf = []
        self.yf = []
        self.thetaf = []

class Elem_:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0

class Vehicle_:
    def __init__(self) -> None:
        self.lw = 2.8 # wheelbase
        self.lf = 0.96 # front hang length
        self.lr = 0.929 # rear hang length
        self.lb = 1.942 # width
             
class Params_:
    def __init__(self) -> None:
        self.task=Task_()
        self.Nobs = 0
        self.obs = []
        self.vehicle=Vehicle_()
    


params_ = Params_()



class Params_:
    def __init__(self) -> None:
        self.vehicle_wheelbase = 2.8    # L_W,wheelbase of the ego vehicle (m)
        # L_F,front hang length of the ego vehicle (m)
        self.vehicle_front_hang = 0.96
        # L_R,rear hang length of the ego vehicle (m)
        self.vehicle_rear_hang = 0.929
        self.vehicle_width = 1.942      # width of the ego vehicle (m)
        # length of the ego vehicle (m)
        self.vehicle_length = self.vehicle_wheelbase + \
            self.vehicle_front_hang + self.vehicle_rear_hang
        self.radius = math.hypot(
            0.25 * self.vehicle_length, 0.5 * self.vehicle_width)
        self.r2x = 0.25 * self.vehicle_length - self.vehicle_rear_hang
        self.f2x = 0.75 * self.vehicle_length - self.vehicle_rear_hang



def CreateVehiclePolygon(x, y, theta, resolution):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    params_.vehicle.lw = 2.8 # wheelbase
    params_.vehicle.lf = 0.96 # front hang length
    params_.vehicle.lr = 0.929 # rear hang length
    params_.vehicle.lb = 1.942 # width
    vehicle_half_width = params_.vehicle.lb * 0.5
    AX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta - vehicle_half_width * sin_theta
    BX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta + vehicle_half_width * sin_theta
    CX = x - params_.vehicle.lr * cos_theta + vehicle_half_width * sin_theta
    DX = x - params_.vehicle.lr * cos_theta - vehicle_half_width * sin_theta
    AY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta + vehicle_half_width * cos_theta
    BY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta - vehicle_half_width * cos_theta
    CY = y - params_.vehicle.lr * sin_theta - vehicle_half_width * cos_theta
    DY = y - params_.vehicle.lr * sin_theta + vehicle_half_width * cos_theta
    V.x = [AX]
    V.x.extend(np.linspace(AX, BX, resolution))
    V.x.append(BX)
    V.x.extend(np.linspace(BX, CX, resolution))
    V.x.append(CX)
    V.x.extend(np.linspace(CX, DX, resolution))
    V.x.append(DX)
    V.x.extend(np.linspace(DX, AX, resolution))
    V.y = [AY]
    V.y.extend(np.linspace(AY, BY, resolution))
    V.y.append(BY)
    V.y.extend(np.linspace(BY, CY, resolution))
    V.y.append(CY)
    V.y.extend(np.linspace(CY, DY, resolution))
    V.y.append(DY)
    V.y.extend(np.linspace(DY, AY, resolution))
    return V


if __name__ == "__main__":
    for id in range(1,1):
        with open('./dataset/task/%d.csv'%id, 'r') as f:
            reader = csv.reader(f)
            tmp = list(reader)
            print(tmp)
            tmp = list(reader)
            v = [float(i) for i in tmp[0]]
        params_.task.x0 = v[0]
        params_.task.y0 = v[1]
        params_.task.theta0 = v[2]
        params_.task.xf = v[3]
        params_.task.yf = v[4]
        params_.task.thetaf = v[5]
        params_.Nobs = int(v[6])
        num_vertexes = v[(6 + 1):(7 + params_.Nobs)]
        v[0:(7 + params_.Nobs)] = []

        params_.obs = []

        for ii in range (0,params_.Nobs):
            x = []
            y = []

            for jj in range(0,int(num_vertexes[ii])):
                x.append(v[0])
                y.append(v[1])
                v[0:2] = []
            elem = Elem_() 
            x.append(x[0])
            elem.x = x
            y.append(y[0])
            elem.y = y
            params_.obs.append(elem)
            
        plt.figure()
 
        plt.get_current_fig_manager().full_screen_toggle() # toggle fullscreen mode
        xmin = min(params_.task.x0, params_.task.xf) - 8
        xmax = max(params_.task.x0, params_.task.xf) + 8
        ymin = min(params_.task.y0, params_.task.yf) - 8
        ymax = max(params_.task.y0, params_.task.yf) + 8
        plt.xlim(xmin,xmax)
        plt.ylim(ymin,ymax)
        plt.gca().set_aspect('equal', adjustable='box')

        for jj in range(1,params_.Nobs):
            V = params_.obs[jj]
            plt.fill(V.x, V.y, facecolor='k',alpha=0.5)

        plt.arrow(params_.task.x0, params_.task.y0, math.cos(params_.task.theta0), math.sin(params_.task.theta0),width = 0.2,color="gold")
        plt.arrow(params_.task.xf, params_.task.yf, math.cos(params_.task.thetaf), math.sin(params_.task.thetaf),width = 0.2,color="gold")
        plt.xlabel('x / m', fontsize = 17, fontname = 'Arial Narrow')
        plt.ylabel('y / m', fontsize = 17, fontname = 'Arial Narrow')
        temp = CreateVehiclePolygon(params_.task.xf, params_.task.yf, params_.task.thetaf, 2)
        plt.plot(temp.x, temp.y,linestyle='--', color='red')
        temp = CreateVehiclePolygon(params_.task.x0, params_.task.y0, params_.task.theta0, 2)
        plt.plot(temp.x, temp.y,linestyle='--', color='green')
        # plt.grid()
    plt.show()
