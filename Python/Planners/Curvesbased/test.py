import cv2
import numpy as np
import globalvar
def CreateCostmap():
    costmap_nd = np.zeros((globalvar.num_nodes_x,globalvar.num_nodes_y,1))
    
    cv2.imshow("dwa", costmap_nd)
    cv2.waitKey()

CreateCostmap()
