import numpy as np

from robot_model_calibration import coordinate_transform
from test_dh_model import RotationMatrix2EulerAngles
from test_tcp_calibration import dft_TBM

def test_coordinate_transform():
    dft_tcp = np.array([57,24,85,1])
    # calc_tcp = np.array([-60.302472,-30.014899,-88.492088,1])
    calc_tcp = np.array([60.302472,30.014899,88.492088,1])
    tcp_in_laser_tracker = coordinate_transform.GetTCPinLaserTracker(dft_tcp, dft_TBM)
    # print(tcp_in_laser_tracker)
    tcp_in_robot_base = coordinate_transform.GetTCPinRobotBaseFrame(calc_tcp)
    # print(tcp_in_robot_base)
    tbm_fit = coordinate_transform.RobotLasertrackerTransform(tcp_in_robot_base,tcp_in_laser_tracker)
    print(type(tbm_fit))
    rot = tbm_fit[0:3,:]
    print(rot)
    U,s,V = np.linalg.svd(rot.T)
    rot_ortho = U @ V

    angle = RotationMatrix2EulerAngles(rot_ortho)
    print(angle)
    print(tbm_fit)

if __name__ == '__main__':
    test_coordinate_transform()



