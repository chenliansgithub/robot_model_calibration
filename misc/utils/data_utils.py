import pandas as pd
import numpy as np
def ReadParameters():
    '''
    Import basic D-H data.

    Args:

    Returns:
    df_theta: class 'numpy.ndarray' with shape Mx6
    Theta table of some points in the workspace.
    df_dh_nominal: class 'numpy.ndarray' with shape 6x5
    Nominal D-H parameters pertaining to theta,a,d,alpha,beta
    df_dh_delta: class 'numpy.ndarray' with shape 6x5
    Nominal D-H parameters error

    '''
    df_theta = pd.read_excel('data/excel/theta_table.xlsx', header=None)
    # 转换成弧度制
    df_theta = df_theta.to_numpy() * np.pi / 180
    # Read the nominal D-H parameters.
    df_dh_nominal = pd.read_excel('data/excel/dh_original.xlsx').to_numpy()
    # Read the error of D-H model, theta,alpha,beta are in radian system.
    df_dh_delta = pd.read_excel('data/excel/dh_delta.xlsx').to_numpy()  
    return df_theta, df_dh_nominal, df_dh_delta

if __name__ == '__main__':
    df_theta,df_nominal,df_dh_delta=ReadParameters()
    print(df_theta,df_nominal,df_dh_delta,sep='\n')
    print(df_dh_delta+df_nominal,sep='\n')
    print(df_theta.ndim,type(df_theta),type(df_nominal),type(df_dh_delta),sep='\n')