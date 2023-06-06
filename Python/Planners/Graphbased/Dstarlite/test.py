import warnings
import matplotlib.pyplot as plt
import math
from joblib import parallel_backend
from matplotlib.pyplot import flag
import numpy as np
import time

from random import random as rand
from matplotlib.path import Path
from numpy.linalg import norm as norm

import globalvar
from globalvar import vclass
from PlanDStarPath import Dstar
    
# [x,y,theta,length,completness] = PlanAStarPath（）
time_start = time.time()
pf = Dstar()

pf.replan()

current_obstacle = vclass()
current_obstacle.x = np.array([10, 10, 11, 11])
current_obstacle.y = np.array([10, 11, 11, 10])
pf.add_obstacle(current_obstacle)
Is_complete=pf.replan()
if(Is_complete):
    [x, y, theta, path_length, completeness_flag] = pf.get_path()


time_end = time.time()
print('time cost', time_end-time_start, 's')