#coding=UTF-8
import numpy as np
import pandas as pd

from robot_model_calibration import dh_model
from robot_model_calibration import coordinate_transform
from misc.utils.data_utils import ReadParameters

def GetTCPinLaserTracker(dft_tcp, dft_robot2lasertracker):
    '''
    Calculate position of the target ball in laser tracker Frame.

    Args:
    dft_tcp: class 'numpy.ndarray' of shape 1x4
    The default TCP position in FCS(frange coordinate system) of real robot D-H model.
    dft_robot2lasertracker: class 'numpy.ndarray' of shape 4x4
    The default homogeneous transform matrix from the laser tracker base to the robot,
        and the vetor in robot base(R) and laser traker(M) satisfies $^RP = _M^BT\cdot ^MP$
    
    Returns: class 'numpy.ndarray' of shape Mx4
    A numpy.ndarray pertaining to the real position of the target ball.
        M is the number of points measured and the forth number is 1 in each row.

    Raises:
    '''
    # Read the angle of joints.
    df_theta,df_dh_nominal,df_dh_delta = ReadParameters()
    # Calculate the real D-H parameters.
    real_dh_table = df_dh_nominal + df_dh_delta
    # Split real_dh_table into:
    # rdelta_theta, consists of nominal initiate theta offset,[0,-\pi/2,0,0,0,\pi/2] for Kawasaki BA006N robot.
    # rd,ra,ralpha,rbete, real D-H parameters of the rest.
    rdelta_theta = real_dh_table[:,0]
    rd = real_dh_table[:,1]
    ra = real_dh_table[:,2]
    ralpha = real_dh_table[:,3]
    rbeta = real_dh_table[:,4]

    # Calculate real TCP position in robot base system and transform into laser tracker system
    tcp_in_laser_tracker = np.zeros([len(df_theta),4])
    for i in range(len(df_theta)):
        # rtheta = [i+j for i,j in zip(df_theta[i],rdelta_theta)]
        rtheta = df_theta[i,:] + rdelta_theta
        temp_model_fcs = dh_model.RobotDHModel(rtheta,rd,ra,ralpha,rbeta).GetFCS()
        dft_robot2laser_tracker_inv = np.linalg.pinv(dft_robot2lasertracker)
        tcp_in_laser_tracker[i] = dft_robot2laser_tracker_inv @ temp_model_fcs @ dft_tcp
    return tcp_in_laser_tracker


def GetTCPinRobotBaseFrame(calc_tcp):
    '''
    Calculate position of the target ball in the robot system with nominal D-H parameters.

    Args:
    calc_tcp: class 'numpy.ndarray' of shape 1x4
    The  TCP position in FCS(frange coordinate system) of nominal robot D-H model.
    
    Returns: class 'numpy.ndarray' of shape Mx4
    A numpy.ndarray pertaining to the real position of the target ball.
        M is the number of points measured and the forth number is 1 in each row.

    Raises:
    '''
    # Read the angle of joints.
    df_theta,nominal_dh_thble,__ = ReadParameters()
    # Split real_dh_table into:
    # rdelta_theta, consists of nominal initiate theta offset,[0,-\pi/2,0,0,0,\pi/2] for Kawasaki BA006N robot.
    # rd,ra,ralpha,rbete, real D-H parameters of the rest.
    ndelta_theta = nominal_dh_thble[:,0]
    nd = nominal_dh_thble[:,1]
    na = nominal_dh_thble[:,2]
    nalpha = nominal_dh_thble[:,3]
    nbeta = nominal_dh_thble[:,4]

    # Calculate real TCP position in robot base system and transform into laser tracker system
    tcp_in_robot_base = np.zeros([len(df_theta),4])
    for i in range(len(df_theta)):
        # rtheta = [i+j for i,j in zip(df_theta[i],rdelta_theta)]
        ntheta = df_theta[i,:] + ndelta_theta
        temp_model_fcs = dh_model.RobotDHModel(ntheta,nd,na,nalpha,nbeta).GetFCS()
        tcp_in_robot_base[i] = temp_model_fcs @ calc_tcp
    return tcp_in_robot_base
    


def RobotLasertrackerTransform(pos_robot_base,pos_lasertracker):
    '''
    Fit the homogeneous transform matrix from the robot base system to the laser tracker
    Args:
    pos_robot_base: class 'numpy.ndarray' of shape Mx4
    Position of the target ball in robot base system pertaining to the nominal D-H parameters.
    pos_lasertracker: class 'numpy.ndarray' of shape Mx4

    Returns:
    tbm_fit:class 'numpy.ndarray' of shape 4x3
    The matrix pertaining to the transform matrix.
    \begin{bmatrix}
        n_x& n_y & n_z\\
        o_x& o_y & o_z\\
        a_x& a_y & a_z\\
        T_x& T_y & T_z
    \end{bmatrix}
    '''
    pos_robot_base= np.delete(pos_robot_base,3,1)
    tbm_fit = np.linalg.pinv(pos_lasertracker.T @ pos_lasertracker) @ pos_lasertracker.T @pos_robot_base
    return tbm_fit

def main():
    pass
if __name__ == '__main__':
    main()