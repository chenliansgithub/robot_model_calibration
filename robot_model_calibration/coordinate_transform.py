import numpy as np
'''

'''
def RobotLasertrackerTransform(pos_robot,pos_lasertracker):
    '''
    pos_robot:[[x1,y1,z1],[x2,y2,z2],...]
    pos_laertracker:[[x1,y1,z1],[x2,y2,z2],...]

    result:[[nx,ny,nz],[ox,oy,oz][ax,ay,ax],[tx,ty,tz]]
    '''
    for i in len(pos_robot):
        pos_robot[i].append(1)
    A = np.matrix(pos_robot).T
    pos_lasertracker = np.matrix(pos_lasertracker)
    return np.linalg.pinv(A.T @ A) @ A.T @pos_lasertracker

