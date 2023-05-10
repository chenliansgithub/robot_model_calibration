import sympy as sym
import numpy as np

def TransX(dx):
        return sym.Matrix([
        [1, 0, 0, dx],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

def TransY(dy):
        return sym.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, dy],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

def TransZ(dz):
        return sym.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
        ])

def RotX(c):
        return sym.Matrix([
        [1, 0, 0, 0], 
        [0, sym.cos(c), -sym.sin(c), 0],
        [0, sym.sin(c),sym.cos(c), 0], 
        [0, 0, 0, 1]
        ])
    
def RotY(b):
    return sym.Matrix([
    [sym.cos(b), 0, sym.sin(b), 0], 
    [0, 1, 0, 0],
    [-sym.sin(b), 0,sym.cos(b), 0], 
    [0, 0, 0, 1]
    ])

def RotZ(a):
    return sym.Matrix([
    [sym.cos(a), -sym.sin(a), 0, 0], 
    [sym.sin(a), sym.cos(a), 0, 0],
    [0, 0, 1, 0], 
    [0, 0, 0, 1]
    ])
def Inv_Ti(Ti):
    T_inv = sym.eye(4)
    T_inv[0:3,0:3] = Ti[0:3,0:3].T
    T_inv[0:3,3] = -Ti[0:3,0:3].T * Ti[0:3,3]
    return T_inv    

if __name__ == "__main__":
    theta_i = sym.Symbol('theta_i')
    d_i = sym.Symbol('d_i')
    a_i = sym.Symbol('a_i')
    alpha_i = sym.Symbol('alpha_i')

    Delta_theta_i = sym.Symbol('Delta_theta_i')
    Delta_d_i = sym.Symbol('Delta_d_i')
    Delta_a_i = sym.Symbol('Delta_a_i')
    Delta_alpha_i = sym.Symbol('Delta_alpha_i')

    T_i = RotZ(theta_i) * TransZ(d_i) * TransX(a_i) * RotX(alpha_i)
    # print(sym.latex(sym.simplify(T_i)))
    T_i_inv = Inv_Ti(T_i)
    # print(sym.latex(sym.simplify(T_i_inv)))

    Ti_diff_theta_i = sym.diff(T_i,theta_i)
    Ti_diff_d_i = sym.diff(T_i,d_i)
    Ti_diff_a_i = sym.diff(T_i,a_i)
    Ti_diff_alpha_i = sym.diff(T_i,alpha_i)
    
    # print(sym.latex(sym.simplify(Ti_diff_theta_i)))
    # print(sym.latex(sym.simplify(Ti_diff_d_i)))
    # print(sym.latex(sym.simplify(Ti_diff_a_i)))
    # print(sym.latex(sym.simplify(Ti_diff_alpha_i)))
    A_theta_i = Ti_diff_theta_i * T_i_inv
    A_d_i = Ti_diff_d_i * T_i_inv
    A_a_i = Ti_diff_a_i * T_i_inv
    A_alpha_i = Ti_diff_alpha_i * T_i_inv
    delta_T_i = A_theta_i * Delta_theta_i + A_d_i * Delta_d_i + A_a_i * Delta_a_i + A_alpha_i * Delta_alpha_i
    print(sym.latex(sym.simplify(delta_T_i)))
    # print(sym.latex(sym.simplify(A_theta_i)))
    # print(sym.latex(sym.simplify(A_d_i)))
    # print(sym.latex(sym.simplify(A_a_i)))
    # print(sym.latex(sym.simplify(A_alpha_i)))