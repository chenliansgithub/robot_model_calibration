import numpy as np
import sympy as sym
from scipy.optimize import least_squares

import dh_model
from misc.utils.data_utils import ReadParameters

class ErrDHModel():
    '''
    Define a class to establish the differentiable D-H model using sympy.
    '''
    def __init__(self, num_theta,num_d,num_a,num_alpha,num_beta):
        '''
        Initiate the D-H numerical parameters.
        '''
        self.num_theta = num_theta
        self.num_d = num_d
        self.num_a = num_a
        self.num_alpha = num_alpha
        self.num_beta = num_beta
    # Define the differentiable D-H parameters
    theta = sym.symbols('theta1:7')
    d = sym.symbols('d1:7')
    a = sym.symbols('a1:7')
    alpha = sym.symbols('alpha1:7')
    beta = sym.symbols('beta1:7')

    # Define the basic transform matrixes
    def TransX(self,dx):
        return sym.Matrix([[1, 0, 0, dx], 
                           [0, 1, 0, 0], 
                           [0, 0, 1, 0], 
                           [0, 0, 0,1]])
    def TransY(self,dy):
        return sym.Matrix([[1, 0, 0, 0], 
                           [0, 1, 0, dy], 
                           [0, 0, 1, 0], 
                           [0, 0, 0,1]])
    def TransZ(self,dz):
        return sym.Matrix([[1, 0, 0, 0], 
                           [0, 1, 0, 0], 
                           [0, 0, 1, dz], 
                           [0, 0, 0,1]])
    def RotX(self,theta):
        return sym.Matrix([[1, 0, 0, 0], 
                            [0, sym.cos(theta), -sym.sin(theta), 0],
                            [0, sym.sin(theta),sym.cos(theta), 0], 
                            [0, 0, 0, 1]])
    def RotY(self,theta):
        return sym.Matrix([[sym.cos(theta), 0, sym.sin(theta), 0], 
                            [0, 1, 0, 0],
                            [-sym.sin(theta), 0,sym.cos(theta), 0], 
                            [0, 0, 0, 1]])
    def RotZ(self,theta):
        return sym.Matrix([[sym.cos(theta), -sym.sin(theta), 0, 0],
                            [sym.sin(theta), sym.cos(theta), 0, 0], 
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    def DiffRobotModel(self):
        '''
        
        '''
        T = []
        T06 = sym.eye(4)
        A_theta = []
        A_alpha = []
        A_a = []
        A_d = []
        A_beta = []
        for i in range(1):
            temp_T = (self.RotZ(self.theta[i]) 
            * self.TransZ(self.d[i]) 
            * self.TransX(self.a[i]) 
            * self.RotX(self.alpha[i]) 
            * self.RotY(self.beta[i]))
            T06 = T06 * temp_T
            T.append(temp_T)
        # print(sym.simplify(T06[0,3].subs({self.beta[0]:0,self.beta[2]:0,self.beta[3]:0,self.beta[4]:0,self.beta[5]:0})))# 这么做算不出来
        # print('\n\n')
        # print(T06[1,3])
        # print('\n\n')
        # print(T06[2,3])
            
        
            print('temp_T',sym.simplify(temp_T))
            # Calculate the frange to the base
            # T_base2frange = T_base2frange * temp_T
            # Store all the D-H parameters
            T.append(temp_T)

            temp_T_inv = sym.simplify(temp_T.inv())
            # Calculate A_theta
            temp_theta = sym.simplify(temp_T_inv * sym.diff(temp_T,self.theta[i]))
            print('A_theta',temp_theta)
            A_theta.append(temp_theta)

            # Calculate A_d
            temp_d = sym.simplify(temp_T_inv * sym.diff(temp_T,self.d[i]))
            print('A_d',temp_d)
            A_d.append(temp_d)

            # Calculate A_a
            temp_a = sym.simplify(temp_T_inv * sym.diff(temp_T,self.a[i]))
            print('A_a',temp_a)
            A_a.append(temp_a)

            # Calculate A_alpha
            temp_alpha = sym.simplify(temp_T_inv * sym.diff(temp_T, self.alpha[i]))
            print('A_alpha',temp_alpha)
            A_alpha.append(temp_alpha)

            # Calculate A_beta
            temp_beta = sym.simplify(temp_T_inv * sym.diff(temp_T,self.beta[i]))
            print('A_beta',temp_beta)
            A_beta.append(temp_beta)
            
            
        
        
        # print(T_base2frange[0:1,3])
        # print(T_base2frange)
        # T01_beta0_subs = T01.subs({self.beta[0]:0})
        # print('T01_beta0_subs', T01_beta0_subs)
        # print('T01_beta0_subs_inv', sym.simplify(T01_beta0_subs.inv()))
        # T01_beta0_diff = sym.diff(T01, self.beta[0])
        # T01_beta0_diff_subs = T01_beta0_diff.subs({self.beta[0]:0})
        # print('T01_beta0_diff_subs', T01_beta0_diff_subs)
        # # print(T01_beta0_diff_subs)
        # D_beta0 = T01_beta0_diff_subs * sym.simplify(T01_beta0_subs.inv())
        # print("D_beta0", D_beta0)
        # print(D_beta0.simplify.evalf)
        # print(T01)
        # T02 = (self.RotZ(self.theta[]) 
        # * self.TransZ(self.d[0]) 
        # * self.TransX(self.a[0]) 
        # * self.RotX(self.alpha[0]) 
        # * self.RotY(self.beta[0]))
        # T01_inv = T01.inv()
        # print(T01_inv)
        # print(T01_inv*sym.diff(T01,self.theta[0]))
        return T
    


def main():
    a = ErrDHModel(0,0,0,0,0)
    T = a.DiffRobotModel()
    

if __name__ == '__main__':
    main()