import numpy as np
import sympy as sym
from scipy.optimize import leastsq

from robot_model_calibration.dh_model import RobotDHModel
# dft default
# r real

# # 返回的是实际D-H模型下默认的TCP点在激光跟踪仪坐标系下的坐标，np.pos = [x, y, z, 1]
# def GetPosOnLaserTrackerFrame(real_robot_model:RobotDHModel,dft_TBM,dft_tcp):
#     # 机器人实际的法兰末端坐标系
#     r_fcs = real_robot_model.GetFCS()
#     np.pos = np.linalg.pinv(dft_TBM) @ r_fcs @ dft_tcp
#     return np.pos


# TCP标定,工作空间中几个不同的位置，n_fcs_matrix[i]和laser_tracker_pos[i]之间是对应起来的
def TCPCalibration(start_tcp, nominal_fcs_matrix, laser_tracker_pos):
    '''
    start_tcp:[x,y,z,1]
    laser_tracker_pos:[x,y,z]
    '''
    diff_matrix = []
    diff_pos = []
    for i in range(len(nominal_fcs_matrix)-1):
        diff_matrix.append(nominal_fcs_matrix[i+1] - nominal_fcs_matrix[i])
        diff_pos.append([x-y for x,y in zip(laser_tracker_pos[i+1],laser_tracker_pos[i])])
    
    # print(diff_matrix)
    a = []
    b = []
    c = []
    d = []
    
    distance_in_laser_tracker = []
    # print(diff_matrix)
    
    for i in range(len(diff_matrix)):
        # print(type(diff_matrix[i]))
        # print(diff_matrix[0])
        # print(diff_matrix[i][0,0])
        den = 0
        for j in range(len(diff_matrix[0])):
            # for k in range(len(diff_matrix[0][0][0][0][0])):matrix要用[1,2]这样的tuple去索引
            for k in range(diff_matrix[0].shape[1]):
                den += (diff_matrix[i][j,k] * start_tcp[k])**2
                # print(len(diff_matrix[i][j]))
                # print(diff_matrix[i][j])
        den = np.sqrt(den)
        
        num_a = 0
        for j in range(len(diff_matrix[0])):
            temp_a = 0
            for k in range(diff_matrix[0].shape[1]):
                temp_a += diff_matrix[i][j,k]*start_tcp[k]
            num_a += diff_matrix[i][j,0]*temp_a
        a.append(num_a/den)
        
        num_b = 0
        for j in range(len(diff_matrix[0])):
            temp_b = 0
            for k in range(diff_matrix[0].shape[1]):
                temp_b += diff_matrix[i][j,k]*start_tcp[k]
            num_b += diff_matrix[i][j,1]*temp_b
        b.append(num_b/den)
        
        num_c = 0
        for j in range(len(diff_matrix[0])):
            temp_c = 0
            for k in range(diff_matrix[0].shape[1]):
                temp_c += diff_matrix[i][j,k]*start_tcp[k]
            num_c += diff_matrix[i][j,2]*temp_c
        c.append(num_c/den)
        
        d.append(den - np.linalg.norm(diff_pos[i]))
        distance_in_laser_tracker.append(np.linalg.norm(diff_pos[i]))
    # print(distance_in_laser_tracker)
    
    t = np.matrix([a,b,c])
    print(t)
    t = t.T
    d = np.matrix(d)
    temp = np.linalg.pinv(t.T @ t) @ t.T @ d.T
    return temp.T



