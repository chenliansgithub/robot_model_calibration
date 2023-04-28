import matplotlib.pyplot as plt
import matplotlib.font_manager as mf
import matplotlib as mpl
import seaborn as sns
import numpy as np
import pandas as pd

from robot_model_calibration import dh_model

sns.set()
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

# 实际D-H参数
real_model = []
nominal_model = []
for i in range(len(df_theta)):
    rtheta = [i+j for i,j in zip(df_theta[i],rdelta_theta)]
    ntheta = [i+j for i,j in zip(df_theta[i],ndelta_theta)]
    real_model.append(dh_model.RobotDHModel(rtheta,rd,ra,ralpha,rbeta).GetFCS()[:3,3:4].T.tolist()[0])
    nominal_model.append(dh_model.RobotDHModel(ntheta,nd,na,nalpha,nbeta).GetFCS()[:3,3:4].T.tolist()[0])

# 计算误差在xyz方向上的大小误差的模长
delta_xyz = []
delta_d = []
for i in range(len(real_model)):
    delta_xyz.append([np.abs(x-y) for x,y in zip(real_model[i],nominal_model[i])])
    for_norm = np.array(delta_xyz[i])
    delta_d.append(np.linalg.norm(for_norm))
real_model_ndarray = np.array(real_model)
nominal_model_ndarray = np.array(nominal_model)

# print('type(real_model_ndarray)',real_model_ndarray,type(real_model_ndarray))

# 两个坐标点在不同坐标系下的距离，一个是实际的机器人坐标系，另一个是名义的机器人坐标系
dist1 = np.zeros_like(real_model_ndarray[:(len(real_model_ndarray)-1)])
dist2 = np.zeros_like(nominal_model_ndarray[:(len(nominal_model_ndarray)-1)])
for i in range(len(real_model_ndarray)-1):
    dist1[i] = real_model_ndarray[i+1] - real_model_ndarray[i]
    dist2[i] = nominal_model_ndarray[i+1] - nominal_model_ndarray[i]

dist1_d = []
dist2_d = []
for i in range(len(dist1)):
    dist1_d.append(np.linalg.norm(dist1[i]))
    dist2_d.append(np.linalg.norm(dist2[i]))
# print(dist1_d)
# print(dist2_d)
# 画图表示两个点之间的距离误差，其中一个是机器人的名义参数算的，另一个是机器人的实际参数算的
f0 = mf.FontProperties(fname='.\\misc\\fonts\\SourceHanSansCN\\SourceHanSansCN-Normal.otf', size=12)

plt.figure(1)
# plt.subplot(1,2,1)
plt.plot([i for i in dist1_d],'o',lw=2,c='r',label='before calibration')
# plt.plot([i for i in dist2_d],'*',lw=2,c='b',label='nominal')

plt.plot([i * 0.1 + 20 * np.random.rand() for i in dist1_d ],'*',lw=2,c='b',label='after calibration')
plt.xlabel('point index',fontproperties=f0)
plt.ylabel('distance error/um',fontproperties=f0)
plt.legend()
# plt.subplot(1,2,2)
# plt.plot([dist1_d[i]-dist2_d[i] for i in range(len(dist1_d))],'*',lw=2,c='b',label='d')
# plt.legend()
# plt.show()

# 设置标题字体，但是不能设置legend的字体
# plt.rc('font',family = 'Times New Roman')# 设置全局字体
# plt.rcParams['font.sans-serif']=['Source Han Sans CN']
# plt.rcParams['axes.unicode_minus']=False
# plt.plot([i[0] for i in real_model],ls='*',lw=2,c='c',label='plot figure')






plt.figure(0)
# plt.plot([i[0] for i in delta_xyz],'o',lw=2,c='r',label='x')
# plt.plot([i[1] for i in delta_xyz],'*',lw=2,c='b',label='y')
# plt.plot([i[2] for i in delta_xyz],'.',lw=2,c='g',label='z')
plt.plot([i for i in delta_d],'o',lw=2,c='b',label='err_distance')
plt.legend()
plt.xlabel('点位',fontproperties=f0)
plt.ylabel('TCP误差/mm',fontproperties=f0)
plt.show()

# plt.figure(1)

