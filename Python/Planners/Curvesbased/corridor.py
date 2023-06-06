import numpy as np
import globalvar
from main_unstructure import checkObj_linev, distance


class bounding_box(object):
    def __init__(self, center, margin=0.01):
        self.P1 = np.array([center[0] - margin, center[1] + margin])
        self.P2 = np.array([center[0] + margin, center[1] + margin])
        self.P3 = np.array([center[0] - margin, center[1] - margin])
        self.P4 = np.array([center[0] + margin, center[1] - margin])
        self.center = center

    # +1 to change
    def is_Overlap(self, new_box):
        if new_box.P3[0] >= (self.P2[0] + 1) or \
                new_box.P3[1] >= (self.P2[1] + 1) or \
                self.P3[0] >= (new_box.P2[0] + 1) or \
                self.P3[1] >= (new_box.P2[1] + 1):
            return False
        return True

    def inflate_box(self, step=0.3, max_inflate_iter=1000):
        has_P2_x = False
        has_P2_y = False
        has_P3_x = False
        has_P3_y = False
        temp_P2 = self.P2
        temp_P3 = self.P3
        for j in range(max_inflate_iter):
            if not has_P2_x or not has_P2_y:
                if not has_P2_x:
                    temp_P2[1] += step
                    temp_P1 = np.array([temp_P3[0], temp_P2[1]])
                    for obstacle in globalvar.obstacles_[0]:
                        obs = np.vstack((obstacle.x, obstacle.y))
                        if (temp_P2[1] >= globalvar.planning_scale_.ymax or \
                                checkObj_linev(temp_P1, temp_P2, obs)):
                            has_P2_x = True
                            break
                    if has_P2_x:
                        temp_P2[1] -= step

                if not has_P2_y:
                    temp_P2[0] += step
                    temp_P4 = np.array([temp_P2[0], temp_P3[1]])
                    for obstacle in globalvar.obstacles_[0]:
                        obs = np.vstack((obstacle.x, obstacle.y))
                        if (temp_P2[0] >= globalvar.planning_scale_.xmax or \
                                checkObj_linev(temp_P2, temp_P4, obs)):
                            has_P2_y = True
                            break
                    if has_P2_y:
                        temp_P2[0] -= step
            if not has_P3_x or not has_P3_y:
                if not has_P3_x:
                    temp_P3[1] -= step
                    temp_P4 = np.array([temp_P2[0], temp_P3[1]])
                    for obstacle in globalvar.obstacles_[0]:
                        obs = np.vstack((obstacle.x, obstacle.y))
                        if (temp_P3[1] <= globalvar.planning_scale_.ymin or \
                                checkObj_linev(temp_P3, temp_P4, obs)):
                            has_P3_x = True
                            break

                    if has_P3_x:
                        temp_P3[1] += step
                if not has_P3_y:
                    temp_P3[0] -= step
                    temp_P1 = np.array([temp_P3[0], temp_P2[1]])
                    for obstacle in globalvar.obstacles_[0]:
                        obs = np.vstack((obstacle.x, obstacle.y))
                        if (temp_P3[0] <= globalvar.planning_scale_.xmin or \
                                checkObj_linev(temp_P3, temp_P1, obs)):
                            has_P3_y = True
                            break
                    if has_P3_y:
                        temp_P3[0] += step

            if has_P2_x and has_P2_y and has_P3_x and has_P3_y:
                break

        self.P2[1] = temp_P2[1]
        self.P1[1] = temp_P2[1]

        self.P2[0] = temp_P2[0]
        self.P4[0] = temp_P2[0]

        self.P3[1] = temp_P3[1]
        self.P4[1] = temp_P3[1]

        self.P3[0] = temp_P3[0]
        self.P1[0] = temp_P3[0]


class inflate_box(object):
    def __init__(self, path, max_inflate_iter=1000, margin=0.02):
        self.box_list = []
        self.vis_list = []
        self.pt_list = []
        self.path = path
        self.inflate()
        self.time_allocated = self.time_allocate()
        self.max_inflate_iter = max_inflate_iter
        self.margin = margin
        self.corridor = self.get_corridor()

    def get_corridor(self):
        corridor_list = []
        for box in self.box_list:
            xmax = box.P2[0]
            xmin = box.P1[0]
            ymax = box.P2[1]
            ymin = box.P3[1]
            corridor_list.append(np.array([xmin, xmax, ymin, ymax]))
        return np.array(corridor_list)

    def get_box_list(self):
        return self.box_list

    def get_box_size(self):
        return len(self.box_list)

    def simplify_box(self):
        temp = self.box_list
        n = self.get_box_size()
        self.box_list = []
        print('n is ', n)
        idx_old = 0
        self.box_list.append(temp[idx_old])
        for i in range(1, n):
            if not temp[idx_old].is_Overlap(temp[i]):
                self.box_list.append(temp[i - 1])
                idx_old = i - 1
                if temp[i - 1].is_Overlap(temp[n - 1]):
                    break
        self.box_list.append(temp[n - 1])

    def is_in_box(self, pt):
        if self.get_box_size() == 0:
            return False
        if self.box_list[-1].P1[0] <= pt[0] <= self.box_list[-1].P2[0] and \
                self.box_list[-1].P4[1] <= pt[1] <= self.box_list[-1].P2[1]:
            return True
        return False

    # need: get_Overlap_center()
    def time_allocate(self, radio=0.3):
        self.pt_list = self.get_Overlap_center()
        time_allocated = []
        for i in range(0, len(self.pt_list) - 1):
            dis = distance(self.pt_list[i + 1], self.pt_list[i])
            t = dis / (globalvar.vehicle_kinematics_.vehicle_v_max * radio)
            time_allocated.append(t)
        return np.array(time_allocated)

    def update_vis_corridor(self):
        self.vis_list = self.box_list

    def inflate(self):
        box_last = bounding_box(self.path[0])
        n = self.path.shape[0]
        for i in range(n):
            if self.is_in_box(self.path[i]):
                continue
            box_now = bounding_box(self.path[i])
            box_now.inflate_box()
            flag = self.delete_box(box_last, box_now)  # 两个box的非交集部分，谁的非交集部分没有点说明它是重复的
            if flag == 1:
                pass
            elif flag == 2:
                if self.get_box_size() > 0:
                    self.box_list.pop()
                while self.get_box_size() > 0:  # 判断之前的box会不会被box_now取代
                    box_last = self.box_list[-1]
                    flag2 = self.delete_box(box_last, box_now)
                    if flag2 != 2:
                        break
                    self.box_list.pop()
                self.box_list.append(box_now)
                box_last = box_now
            else:
                self.box_list.append(box_now)
                box_last = box_now
        self.update_vis_corridor()
        if self.get_box_size() > 1:
            self.simplify_box()

    # need: path and box_list
    def get_Overlap_center(self):
        self.pt_list = []
        self.pt_list.append(self.path[0])
        for i in range(self.get_box_size() - 1):
            x_low = max(self.box_list[i].P1[0], self.box_list[i + 1].P1[0])
            x_up = min(self.box_list[i].P2[0], self.box_list[i + 1].P2[0])
            y_low = max(self.box_list[i].P4[1], self.box_list[i + 1].P4[1])
            y_up = min(self.box_list[i].P2[1], self.box_list[i + 1].P2[1])
            pt = np.array([(x_low + x_up) / 2, (y_low + y_up) / 2])
            self.pt_list.append(pt)
        self.pt_list.append(self.path[-1])
        return self.pt_list

    # need: path
    def delete_box(self, box_last, box_now):
        n = self.path.shape[0]
        has_box_last = False
        has_box_now = False
        for i in range(n - 1, -1, -1):
            Px = self.path[i][0]
            Py = self.path[i][1]
            if not has_box_last:
                if box_last.P1[0] <= Px <= box_last.P2[0] and box_last.P4[1] <= Py <= box_last.P2[1] and (
                        Px < box_now.P1[0] or Px > box_now.P2[0] or Py < box_now.P4[1] or Py > box_now.P2[1]):
                    has_box_last = True
            if not has_box_now:
                if box_now.P1[0] <= Px <= box_now.P2[0] and box_now.P4[1] <= Py <= box_now.P2[1] and (
                        Px < box_last.P1[0] or Px > box_last.P2[0] or Py < box_last.P4[1] or Py > box_last.P2[1]):
                    has_box_now = True
            if has_box_last and has_box_now:
                return 0
        if has_box_last and not has_box_now:
            return 1
        return 2
