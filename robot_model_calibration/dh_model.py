import numpy as np

'''

'''
class RobotDHModel():
    def __init__(self,theta,d,a,alpha,beta):
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
        self.beta = beta
    
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
    
    # 机器人模型
    def GetT01(self):
        return self.RotZ(self.theta[0]) @ self.TransZ(self.d[0]) @ self.TransX(self.a[0]) @ self.RotX(self.alpha[0])
    

    def GetFCS(self):
        T01 = self.RotZ(self.theta[0]) @ self.TransZ(self.d[0]) @ self.TransX(self.a[0]) @ self.RotX(self.alpha[0])
        T12 = self.RotZ(self.theta[1]) @ self.TransZ(self.d[1]) @ self.TransX(self.a[1]) @ self.RotX(
            self.alpha[1]) @ self.RotY(self.beta[1])
        T23 = self.RotZ(self.theta[2]) @ self.TransZ(self.d[2]) @ self.TransX(self.a[2]) @ self.RotX(self.alpha[2])
        T34 = self.RotZ(self.theta[3]) @ self.TransZ(self.d[3]) @ self.TransX(self.a[3]) @ self.RotX(self.alpha[3])
        T45 = self.RotZ(self.theta[4]) @ self.TransZ(self.d[4]) @ self.TransX(self.a[4]) @ self.RotX(
            self.alpha[4])  # 在第六个关节上
        T56 = self.RotZ(self.theta[5]) @ self.TransZ(self.d[5]) @ self.TransX(self.a[5]) @ self.RotX(
            self.alpha[5])  # 到末端法兰盘
        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        return T06
    
    # def GetErrParameter(self):
    #     T = []
    #     T06 = 1
    #     A_theta = []
    #     A_d = []
    #     A_a = []
    #     A_alpha = []
    #     A_beta = []
    #     for i in range(6):
    #         temp_T = (self.RotZ(self.theta[i]) 
    #                   @ self.TransZ(self.d[i]) 
    #                   @ self.TransX(self.a[i]) 
    #                   @ self.RotX(self.alpha[i]) 
    #                   @ self.RotY(self.beta[i]))
    #         T06 = T06 @ temp_T
    #         temp_tehta = np.matrix([[0, -np.cos(self.alpha[i])*np.cos(self.beta[i]), np.sin(self.alpha[i]), self.a[i]*np.sin(self.alpha[i])*np.sin(self.beta[i])], 
    #                                 [np.cos(self.alpha[i])*np.cos(self.beta[i]), 0, np.sin(self.beta[i])*np.cos(self.alpha[i]), self.a[i]*np.cos(self.alpha[i])], 
    #                                 [-np.sin(self.alpha[i]), -np.sin(self.beta[i])*np.cos(self.alpha[i]), 0, -self.a[i]*np.sin(self.alpha[i])*np.cos(self.beta[i])], 
    #                                 [0, 0, 0, 0]])
    #         temp_d = np.matrix([[0, 0, 0, -np.sin(self.beta[i])*np.cos(self.alpha[i])], 
    #                             [0, 0, 0, np.sin(self.alpha[i])], 
    #                             [0, 0, 0, np.cos(self.alpha[i])*np.cos(self.beta[i])], 
    #                             [0, 0, 0, 0]])
    #         temp_a = np.matrix([[0, 0, 0, np.cos(self.beta[i])], 
    #                             [0, 0, 0, 0], 
    #                             [0, 0, 0, np.sin(self.beta[i])], 
    #                             [0, 0, 0, 0]])
    #         temp_alpha = np.matrix([[0, -np.sin(self.beta[i]), 0, 0], 
    #                                 [np.sin(self.beta[i]), 0, -np.cos(self.beta[i]), 0], 
    #                                 [0, np.cos(self.beta[i]), 0, 0], 
    #                                 [0, 0, 0, 0]])
    #         temp_beta = np.matrix([[0, 0, 1, 0], 
    #                                [0, 0, 0, 0], 
    #                                [-1, 0, 0, 0], 
    #                                [0, 0, 0, 0]])
    #         T.append(temp_T)
    #         A_theta.append(temp_tehta)
    #         A_d.append(temp_d)
    #         A_a.append(temp_a)
    #         A_alpha.append(temp_alpha)
    #         A_beta.append(temp_beta)
        
        
        # T01 = self.RotZ(self.theta[0]) @ self.TransZ(self.d[0]) @ self.TransX(self.a[0]) @ self.RotX(self.alpha[0])
        # T12 = self.RotZ(self.theta[1]) @ self.TransZ(self.d[1]) @ self.TransX(self.a[1]) @ self.RotX(
        #     self.alpha[1]) @ self.RotY(self.beta[1])
        # T23 = self.RotZ(self.theta[2]) @ self.TransZ(self.d[2]) @ self.TransX(self.a[2]) @ self.RotX(self.alpha[2])
        # T34 = self.RotZ(self.theta[3]) @ self.TransZ(self.d[3]) @ self.TransX(self.a[3]) @ self.RotX(self.alpha[3])
        # T45 = self.RotZ(self.theta[4]) @ self.TransZ(self.d[4]) @ self.TransX(self.a[4]) @ self.RotX(
        #     self.alpha[4])  # 在第六个关节上
        # T56 = self.RotZ(self.theta[5]) @ self.TransZ(self.d[5]) @ self.TransX(self.a[5]) @ self.RotX(
        #     self.alpha[5])  # 到末端法兰盘
        # T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        return T06