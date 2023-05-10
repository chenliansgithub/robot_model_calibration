import numpy as np

'''

'''
class RobotDHModel():
    def __init__(self,dh_model):
        '''
        :param dh_model: DH参数
        :param theta: 误差角度1x6
        弧度制
        '''
        self.theta = dh_model[:, 0]
        self.d = dh_model[:, 1]
        self.a = dh_model[:, 2]
        self.alpha = dh_model[:, 3]
        self.beta = dh_model[:, 4]
    
    # 基本矩阵变换
    def TransX(self,dx):
        return np.matrix([
        [1, 0, 0, dx],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])
    
    def TransY(self,dy):
        return np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, dy],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])
    
    def TransZ(self,dz):
        return np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
        ])
    
    def RotX(self,c):
        return np.matrix([
        [1, 0, 0, 0], 
        [0, np.cos(c), -np.sin(c), 0],
        [0, np.sin(c),np.cos(c), 0], 
        [0, 0, 0, 1]
        ])
    
    def RotY(self,b):
        return np.matrix([
        [np.cos(b), 0, np.sin(b), 0], 
        [0, 1, 0, 0],
        [-np.sin(b), 0,np.cos(b), 0], 
        [0, 0, 0, 1]
        ])
    
    def RotZ(self,a):
        return np.matrix([
        [np.cos(a), -np.sin(a), 0, 0], 
        [np.sin(a), np.cos(a), 0, 0],
        [0, 0, 1, 0], 
        [0, 0, 0, 1]
        ])
    #计算系数k1,k2,k3
    def GetK(self,theta):
        theta = self.theta+theta
        k1 = [0]*6
        k2 = [0]*6
        k3 = [0]*6
        for i in range(6):
            k1[i]=np.array([-self.d[i]*np.sin(theta[i]),self.d[i]*np.cos(theta[i]),0])
            k2[i]=np.array([np.cos(theta[i]),np.sin(theta[i]),0])
            k3[i]=np.array([0,0,1])
        return k1,k2,k3
    # 得到T[i]和W[i]
    def GetT(self,theta):
        theta = self.theta+theta
        T=[0]*6
        for i in range(6):
            T[i] = self.RotZ(theta[i]) @ self.TransZ(self.d[i]) @ self.TransX(self.a[i]) @ self.RotX(self.alpha[i])
        return T
    def GetW(self,theta):
        W=[np.eye(4)]*6
        for i in range(6):
            for j in range(6-i):
                W[i] = W[i] @ self.GetT(theta)[j]
        return W
    
    #[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]
    def GetErrorJacobian(self,theta):
        k1,k2,k3 = self.GetK(theta)
        W = self.GetW(theta)
        jacobian = np.zeros((3,24))
        # print(jacobian)
        for i in range(6):
            for j in range(3):
                # print(W[i][0:3,j].T,W[i][0:3,3].T,sep='\n')
                jacobian[j,4*i]=np.cross(W[i][0:3,3].T,W[i][0:3,j].T)@k3[i]
                jacobian[j,4*i+1]=W[i][0:3,j].T@k3[i]
                jacobian[j,4*i+2]=W[i][0:3,j].T@k2[i]
                jacobian[j,4*i+3]=W[i][0:3,j].T@k1[i]+np.cross(W[i][0:3,3].T,W[i][0:3,j].T)@k2[i]
        return jacobian
    
    def GetFCS(self):
        # print(self.theta[0],self.d[0],self.a[0],self.alpha[0])
        T01 = self.RotZ(self.theta[0]) @ self.TransZ(self.d[0]) @ self.TransX(self.a[0]) @ self.RotX(self.alpha[0])
        # print(T01)
        T12 = self.RotZ(self.theta[1]) @ self.TransZ(self.d[1]) @ self.TransX(self.a[1]) @ self.RotX(
            self.alpha[1]) @ self.RotY(self.beta[1])
        # print(T01@T12)
        T23 = self.RotZ(self.theta[2]) @ self.TransZ(self.d[2]) @ self.TransX(self.a[2]) @ self.RotX(self.alpha[2])
        # print(T01@T12@T23)
        T34 = self.RotZ(self.theta[3]) @ self.TransZ(self.d[3]) @ self.TransX(self.a[3]) @ self.RotX(self.alpha[3])
        # print(T01@T12@T23@T34)
        T45 = self.RotZ(self.theta[4]) @ self.TransZ(self.d[4]) @ self.TransX(self.a[4]) @ self.RotX(
            self.alpha[4])  # 在第六个关节上
        # print(T01@T12@T23@T34@T45)
        T56 = self.RotZ(self.theta[5]) @ self.TransZ(self.d[5]) @ self.TransX(self.a[5]) @ self.RotX(
            self.alpha[5])  # 到末端法兰盘
        # print(T01@T12@T23@T34@T45@T56)
        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        return T06
    # 把每个关节转动的角度和误差角度加在一起，再把模型自带的角度加上，得到最终的角度
    def GetFCSPos(self,theta):
        '''
        :param dh_model: DH参数
        :param theta: 误差角度1x6
        '''
        theta = self.theta + theta
        T01 = self.RotZ(theta[0]) @ self.TransZ(self.d[0]) @ self.TransX(self.a[0]) @ self.RotX(self.alpha[0])
        T12 = self.RotZ(theta[1]) @ self.TransZ(self.d[1]) @ self.TransX(self.a[1]) @ self.RotX(
            self.alpha[1])
        T23 = self.RotZ(theta[2]) @ self.TransZ(self.d[2]) @ self.TransX(self.a[2]) @ self.RotX(self.alpha[2])
        T34 = self.RotZ(theta[3]) @ self.TransZ(self.d[3]) @ self.TransX(self.a[3]) @ self.RotX(self.alpha[3])
        T45 = self.RotZ(theta[4]) @ self.TransZ(self.d[4]) @ self.TransX(self.a[4]) @ self.RotX(
            self.alpha[4])  # 在第六个关节上
        T56 = self.RotZ(theta[5]) @ self.TransZ(self.d[5]) @ self.TransX(self.a[5]) @ self.RotX(
            self.alpha[5])  # 到末端法兰盘
        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        return T06[0:3, 3]

