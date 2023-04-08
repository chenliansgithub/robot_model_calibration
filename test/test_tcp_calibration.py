import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.font_manager as mf
import seaborn as sns
import numpy as np
import pandas as pd

from robot_model_calibration import dh_model
from robot_model_calibration import tcp_calibration
import test_dh_model


# 读取关节角度
df_theta = pd.read_excel('data/excel/theta_table.xlsx',header=None)
# 转为弧度制
df_theta = df_theta/180*np.pi
# 将datafram转为list
df_theta = df_theta.iloc[:50].values.tolist() 

# 读取原始D-H参数
df_dh_nominal = pd.read_excel('data/excel/dh_original.xlsx')

# 读取参数误差，其中theta,alpha,beta都是弧度制
df_dh_delta = pd.read_excel('data/excel/dh_delta.xlsx')


# 计算实际的dh参数表
real_dh_table = df_dh_nominal + df_dh_delta
# print(real_dh_table)

# 得到实际D-H参数
rdelta_theta = real_dh_table.iloc[:,0].values.tolist()
rd = real_dh_table.iloc[:,1].values.tolist()
ra = real_dh_table.iloc[:,2].values.tolist()
ralpha = real_dh_table.iloc[:,3].values.tolist()
rbeta = real_dh_table.iloc[:,4].values.tolist()

# 名义D-H参数 
ndelta_theta = df_dh_nominal.iloc[:,0].values.tolist()
nd = df_dh_nominal.iloc[:,1].values.tolist()
na = df_dh_nominal.iloc[:,2].values.tolist()
nalpha = df_dh_nominal.iloc[:,3].values.tolist()
nbeta = df_dh_nominal.iloc[:,4].values.tolist()

real_model_fcs = []
nominal_model_fcs = []
for i in range(len(df_theta)):
    rtheta = [i+j for i,j in zip(df_theta[i],rdelta_theta)]
    ntheta = [i+j for i,j in zip(df_theta[i],ndelta_theta)]
    real_model_fcs.append(dh_model.RobotDHModel(rtheta,rd,ra,ralpha,rbeta).GetFCS())
    nominal_model_fcs.append(dh_model.RobotDHModel(ntheta,nd,na,nalpha,nbeta).GetFCS())


# 默认TCP
dft_tcp = np.array([57,24,85,1])

# 初值TCP 
start_tcp = np.array([0,0,0,1])

# 构造机器人和激光跟踪仪之间的转换关系
a = dh_model.RobotDHModel(ndelta_theta,nd,na,nalpha,nbeta)

# 默认激光跟踪仪和机器人基坐标系转换关系
dft_TBM = (a.TransX(2045) 
           @ a.TransY(851) 
           @ a.TransZ(90) 
           @ a.RotZ(np.pi/2.1)
           @ a.RotY(0.1)
           @ a.RotX(0.1))
# print("dft_TBM",dft_TBM)
dft_TBM_inv = np.linalg.pinv(dft_TBM)
# 验证激光跟踪仪和机器人之间的转换关系 test 成功
# print('dft_TBM=',dft_TBM)
# print('dft_rot',dft_TBM[:3,:3])
# print('dft_euler',test_dh_model.RotationMatrix2EulerAngles(dft_TBM[:3,:3]))
# dft_TBM_inv = np.linalg.pinv(dft_TBM)
# print('dft_TBM_inv',dft_TBM_inv)
# print('dft_TBM_inv_euler',test_dh_model.RotationMatrix2EulerAngles(dft_TBM_inv[:3,:3]))

# print(dft_TBM @ dft_TBM_inv @ init_tcp.T)

# 默认TCP和real_model_fcs点乘，得到TCP在机器人坐标系下的实际坐标
laser_tracker_pos = np.array(np.zeros([len(real_model_fcs),4]))
for i in range(len(real_model_fcs)):
    laser_tracker_pos[i] =  dft_TBM_inv @ real_model_fcs[i] @ dft_tcp.T
# print(laser_tracker_pos)
tcp = tcp_calibration.TCPCalibration(start_tcp,nominal_model_fcs,laser_tracker_pos)

sns.set()
def PlotTCPCali():
    for i in range(1):
        tcp_norm = np.array(dft_tcp[:3])
        tcp_norm = np.linalg.norm(tcp_norm)
        dft_tcp_with_norm = np.append(dft_tcp[:3], tcp_norm)
        # print('I am here',dft_tcp)
        # print(type(tcp_norm),tcp_norm,np.linalg.norm(tcp_norm))
        plt.subplot(1,2,1)
        plt.plot(dft_tcp_with_norm,'o', label = '实际tcp')
        dft_tcp_with_norm_list = list(dft_tcp_with_norm)

        temp_tcp = tcp_calibration.TCPCalibration(start_tcp,nominal_model_fcs,laser_tracker_pos)
        # temp_tcp = list(temp_tcp[0])
        temp_tcp_list = []
        for i in range(temp_tcp.shape[1]):
            temp_tcp_list.append(-temp_tcp[0,i])
        print(temp_tcp_list)
        temp_tcp_norm = np.linalg.norm(temp_tcp)
        temp_tcp_list.append(temp_tcp_norm)
        # print('I am here', temp_tcp,temp_tcp_norm)
        # temp_tcp_1 = np.append(temp_tcp[0], -temp_tcp_norm)
        plt.plot(temp_tcp_list,'o', label = '标定得到的tcp')
        index_x = ['x','y','z', 'norm']
        plt.xticks([0,1,2,3],index_x)
        plt.legend()
        plt.subplot(1,2,2)
        plt.plot([j-i for i,j in zip(dft_tcp_with_norm_list,temp_tcp_list)], 'yo', label = '误差')

PlotTCPCali()
index_x = ['x','y','z', 'norm']
plt.xticks([0,1,2,3],index_x)
plt.legend()
plt.show()
print(tcp)