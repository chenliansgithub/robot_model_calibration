import numpy as np
import sympy as sym
from scipy.optimize import leastsq

from dh_model import RobotDHModel
# dft default
# r real

# 返回的是实际D-H模型下默认的TCP点在激光跟踪仪坐标系下的坐标，np.pos = [x, y, z, 1]
def GetPosOnLaserTrackerFrame(real_robot_model:RobotDHModel,dft_TBM,dft_tcp):
    # 机器人实际的法兰末端坐标系
    r_fcs = real_robot_model.GetFCS()
    np.pos = np.linalg.pinv(dft_TBM) @ r_fcs @ [dft_tcp, 1]
    return np.pos


# TCP标定,工作空间中6个不同的位置
def TCPCalibration(start_tcp, n_fcs_matrix, laser_tracker_pos):
    '''
    start_tcp:[x,y,z,1]
    '''
    diff_matrix = []
    diff_pos = []
    for i in range(len(n_fcs_matrix)):
        diff_matrix.append(n_fcs_matrix[i+1] - n_fcs_matrix[i])
        diff_pos.append(laser_tracker_pos[i+1]-laser_tracker_pos[i])
    
    a = []
    b = []
    c = []
    d = []
    
    for i in range(len(diff_matrix)):
        den = 0
        for j in range(len(diff_matrix[0])):
            for k in range(len(diff_matrix[0][0])):
                den += (diff_matrix[i][j][k]*start_tcp[k])**2
        den = np.sqrt(den)
        
        num_a = 0
        for j in range(len(diff_matrix[0])):
            temp_a = 0
            for k in range(len(diff_matrix[0][0])):
                temp_a += diff_matrix[i][j][k]*start_tcp[k]
            num_a += diff_matrix[i][j][0]*temp_a
        a.append(num_a/den)
        
        num_b = 0
        for j in range(len(diff_matrix[0])):
            temp_b = 0
            for k in range(len(diff_matrix[0][0])):
                temp_b += diff_matrix[i][j][k]*start_tcp[k]
            num_a += diff_matrix[i][j][1]*temp_c
        b.append(num_b/den)
        
        num_c = 0
        for j in range(len(diff_matrix[0])):
            temp_c = 0
            for k in range(len(diff_matrix[0][0])):
                temp_c += diff_matrix[i][j][k]*start_tcp[k]
            num_a += diff_matrix[i][j][2]*temp_c
        b.append(num_c/den)
        
        d.append(den - np.linalg.norm(diff_pos[i]))
    
    t = np.matrix[a,b,c].T
    d = np.matrix(d)
    return np.linalg.pinv(t.T @ t) @ t.T @ d













