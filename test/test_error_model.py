import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

from robot_model_calibration import dh_model
from misc.utils.data_utils import ReadParameters

def FlattenList(l):
    '''
    Flatten a list.
    '''
    return [i for item in l for i in item]

def FlattenError(points_error):
    '''
    Flatten the Error form.
    '''
    a = [i.T.tolist()[0] for i in points_error]
    return [i for item in a for i in item]

# 计算误差在xyz方向上的误差
def GetPointsErrorXYZ():
    '''
    Get the error of the points in the workspace.
    '''
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    points_error = []
    for i in range(len(dh_theta)):
        theta = dh_theta[i,:]
        nominal_pos = dh_model.RobotDHModel(dh_nominal).GetFCSPos(theta)
        real_pos = dh_model.RobotDHModel(dh_delta+dh_nominal).GetFCSPos(theta)
        points_error.append(real_pos - nominal_pos)
    return points_error

# 计算误差的模长
def GetPointsErrorNorm():
    '''
    Get the norm of the error of the points in the workspace.
    '''
    points_error = GetPointsErrorXYZ()
    points_error_norm = [np.linalg.norm(i) for i in points_error]
    return points_error_norm


if __name__ == '__main__':
    points_error = GetPointsErrorXYZ()
    print(points_error)
    a = FlattenError(points_error)
    print(a)

