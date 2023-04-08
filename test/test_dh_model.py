
import numpy as np
import math
import pandas as pd
import requests

from robot_model_calibration import dh_model

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# zyx
def RotationMatrix2EulerAngles(R) :

    assert(isRotationMatrix(R))
    R = np.matrix(R)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-3

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return [z, y, x]

# dh 参数
theta0 = [0,-np.pi/2,0,0,0,np.pi]
d = [430,0,0,699.0476,0,115]
a = [164.8701,549.6765,212.9389,0,0,0]
alpha = [-np.pi/2,0,-np.pi/2,np.pi/2,-np.pi/2,0]
beta = [0,0,0,0,0,0]

# 4个不同位置对应的FCS
# [X,Y,Z,A,B,C]
# res_from_demonstrator =[[922.0875,-145.6793,1180.3197,166.1238,34.0793,-178.2910],
#       [1030.2471,144.0002,911.4438,165.1315,4.0287,173.5588],
#       [733.7601,254.3910,1308.4939,-172.4536,28.1774,161.1763],
#       [690.5761,528.7902,1167.3499,-149.4596,13.5819,160.5021]
#       ]

theta = [
        [-8.3904,3.8788,-10.9276,-5.3444,63.3274,6.9619],
        [7.4676,14.2118,-2.1832,4.6619,71.8547,20.8935],
        [17.0466,-4.0912,-14.6422,14.4479,78.4667,5.6805],
        [34.9356,0.9188,-7.6252,18.5939,82.0907,1.8525]
        ]

 
# 计算某个具体位置的D-H参数
# rad -> degree
for i in range(len(theta)):
    theta[i] = [item * np.pi/180 for item in theta[i]]
# 加上原始模型中的偏移偏移
for i in range(len(theta)):
    theta[i] = [x+y for x,y in zip(theta[i],theta0)]

pos = []
rot = []
angle = []
dist = []
for i in range(len(theta)):
    pos.append(dh_model.RobotDHModel(theta[i],d,a,alpha,beta))
    rot.append(pos[i].GetFCS()[:3,:3])
    angle.append(RotationMatrix2EulerAngles(rot[i]))
    dist.append(pos[i].GetFCS()[0:3,3:4])

for i in range(len(angle)):
    angle[i] = [item *180/np.pi for item in angle[i]]

for i in range(len(dist)):
    dist[i] = dist[i].T
    dist[i] = dist[i].tolist()[0]

calc_model = []
for i in range(len(dist)):
    calc_model.append(dist[i] + angle[i])

res_from_demonstrator =[[922.0875,-145.6793,1180.3197,166.1238,34.0793,-178.2910],
      [1030.2471,144.0002,911.4438,165.1315,4.0287,173.5588],
      [733.7601,254.3910,1308.4939,-172.4536,28.1774,161.1763],
      [693.5761,528.7902,1167.3499,-149.4596,13.5819,160.5021]
      ]
# df_1 = pd.DataFrame(res_from_demonstrator)
# df_1.to_excel('data/excel/pos_from_demonstrator.xlsx') 

# print(dist)
# print(calc_model)
should_be_zero = []
for i in range(len(calc_model)):
    should_be_zero.append([i - j for i,j in zip(calc_model[i], res_from_demonstrator[i])])

should_be_zero = np.array(should_be_zero)
np.set_printoptions(precision= 6, suppress=True)
# print(should_be_zero)
# print(' ')

calc_model = np.array(calc_model)
# print(calc_model)

# print(dist)
# print(angle)
print(calc_model)
# df_res_from_demo = pd.
# df_1 = pd.DataFrame(calc_model-res_from_demonstrator)
# df_1.to_excel('data/excel/calc_model.xlsx')
R = np.matrix([[   0.079291  ,  0.990584  , -0.096234],
 [  -0.990827   , 0.086483  ,  0.099382],
 [   0.106621  ,  0.090922  ,  0.991638]])

Rt = np.transpose(R)
shouldBeIdentity = np.dot(Rt, R)
I = np.identity(3, dtype = R.dtype)
n = np.linalg.norm(I - shouldBeIdentity)
print(n)
print(shouldBeIdentity)