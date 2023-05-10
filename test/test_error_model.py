import numpy as np
import sympy as sym
import math
import pandas as pd
import matplotlib.pyplot as plt

from robot_model_calibration import dh_model
from misc.utils.data_utils import ReadParameters
def fk(theta):
    '''
    Forward kinematics.
    '''
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    # 转换成弧度制
    theta = np.array(theta) * np.pi/180
    return dh_model.RobotDHModel(dh_nominal).GetFCSPos(theta)

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


def DrawResults():
    '''
    Get the error of the points in the workspace.
    '''
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    compensate_error = FindError()
    compensate_error = compensate_error.reshape(6,4)
    # 格式上转换成和带有beta参数的一样
    compensate_error_formatted = np.zeros((6,5))
    
    compensate_error_formatted[:,0:4] = compensate_error
    points_error = []
    points_error_after_compensate = []
    for i in range(len(dh_theta)):
        theta = dh_theta[i,:]
        nominal_pos = dh_model.RobotDHModel(dh_nominal).GetFCSPos(theta)
        real_pos = dh_model.RobotDHModel(dh_delta+dh_nominal).GetFCSPos(theta)
        compensate_pos = dh_model.RobotDHModel(dh_delta+compensate_error_formatted).GetFCSPos(theta)
        points_error.append(real_pos - nominal_pos)
        points_error_after_compensate.append(real_pos - compensate_pos)
    points_error_norm = [np.linalg.norm(i) for i in points_error]
    points_error_after_compensate_norm = [np.linalg.norm(i) for i in points_error_after_compensate]
    plt.plot(points_error_norm,label='points_error_norm')
    plt.plot(points_error_after_compensate_norm,label='points_error_after_compensate_norm')
    plt.legend()
    plt.show()
    return points_error
# 计算误差的模长
def GetPointsErrorNorm():
    '''
    Get the norm of the error of the points in the workspace.
    '''
    points_error = GetPointsErrorXYZ()
    points_error_norm = [np.linalg.norm(i) for i in points_error]
    return points_error_norm

def GetPointsErrorListSquareForm():
    '''
    Get the error of the points in the workspace in square form.
    # 将位置三维误差转换为向量形式
    '''
    points_error = GetPointsErrorXYZ()
    points_error_list_square_form = [i.T.tolist()[0] for i in points_error]
    # print(points_error_list_square_form)
    points_error_list_square_form = FlattenList(points_error_list_square_form)
    points_error_list_square_form = np.array(points_error_list_square_form)
    return points_error_list_square_form

def GetcompleteErrorJacobian():
    '''
    Get the complete error jacobian.
    '''
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    error_jacobian = []
    for i in range(len(dh_theta)):
        theta = dh_theta[i,:]
        error_jacobian.append(dh_model.RobotDHModel(dh_nominal).GetErrorJacobian(theta).tolist())
    error_jacobian = FlattenList(error_jacobian)
    error_jacobian = np.array(error_jacobian)
    return error_jacobian

def FindError():
    X = GetcompleteErrorJacobian()
    Y = GetPointsErrorListSquareForm()
    error = np.linalg.inv(np.dot(X.T,X))@X.T@Y
    return error

def testGetK():
    '''
    Get the K.
    '''
    theta = np.array([0,0,0,0,0,0])
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    # RobotDHModel(dh_nominal)的参数dh_nominal可以区分名义模型和实际模型。
    k1,k2,k3 = dh_model.RobotDHModel(dh_nominal).GetK(theta)
    print(k1,k2,k3,sep='\n')

def testGetT(theta):
    '''
    Get the T.
    '''
    # theta = np.array([0,0,0,0,0,0])
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    # RobotDHModel(dh_nominal)的参数dh_nominal可以区分名义模型和实际模型。
    T = dh_model.RobotDHModel(dh_nominal).GetT(theta)
    a = np.eye(4)
    for i in range(len(T)):
        a = a@T[i]
    print('a=',a)
    print('T06=',dh_model.RobotDHModel(dh_nominal).GetFCS())
    
def testGetW():
    '''
    Get the W.
    '''
    theta = np.array([0,0,0,0,0,0])
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    # RobotDHModel(dh_nominal)的参数dh_nominal可以区分名义模型和实际模型。
    W = dh_model.RobotDHModel(dh_nominal).GetW(theta)
    
    # print('FCS=W(1)',dh_model.RobotDHModel(dh_nominal).GetFCS())
    print(W)
def testGetErrorJacobian():
    '''
    Get the error jacobian.
    '''
    theta = np.array([0,0,0,0,0,0])
    dh_theta,dh_nominal,dh_delta=ReadParameters()
    # RobotDHModel(dh_nominal)的参数dh_nominal可以区分名义模型和实际模型。
    error_jacobian = dh_model.RobotDHModel(dh_nominal).GetErrorJacobian(theta)
    # print(error_jacobian)
    # print('error_jacobian=',error_jacobian)
    # print(error_jacobian.shape)

def testGetCompleteErrorJacobian():
    '''
    Get the complete error jacobian.
    '''
    error_jacobian = GetcompleteErrorJacobian()
    print(error_jacobian)
    print('error_jacobian=',error_jacobian)
    print(type(error_jacobian))

def testFindError():
    error = FindError()
    print(error)

if __name__ == "__main__":
    # points_error = GetPointsErrorXYZ()
    # print(points_error)
    # a = FlattenError(points_error)
    # print(a)
    # testGetK()
    # testGetT(np.array([0.0266842497186559,1.88219625292580,-0.244898008339726,
    #                    -0.978623086043229,-1.67136305873706,-0.0727500282800819,]))
    
    # testGetW()
    # testGetErrorJacobian()
    # testGetCompleteErrorJacobian()
    # a = GetPointsErrorListSquareForm()
    # a = FlattenList(a)
    # print(a)
    # testFindError()
    # DrawResults()
    # testGetT()
    # theta = [0.359706380329553,1.12507903382973,1.37265234521387,0.393745965816272,0.295309474362204,0.393745965816272]
    theta1 = [0,0,0,0,0,0]
    theta2 = [-8.3904,3.8788,-10.9276,-5.3444,63.3274,6.9619]
    pos = fk(theta2)
    print(pos)
    # print(pos,T,sep='\n')
    # pass
