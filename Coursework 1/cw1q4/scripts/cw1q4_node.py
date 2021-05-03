#!/usr/bin/env python
###################################
# COMP0127 CW1
# Author:   Douglas Chiang Hou Nam
# SN:       15055142
###################################
import rospy
import numpy        as np

##TODO: Include the header file for the three services
from cw1q4_srv.srv  import quat2rodrigues
from cw1q4_srv.srv  import quat2rodriguesResponse
from cw1q4_srv.srv  import quat2zyx
from cw1q4_srv.srv  import quat2zyxResponse
from cw1q4_srv.srv  import rotmat2quat
from cw1q4_srv.srv  import rotmat2quatResponse

##TODO: Complete these functions
def convert_quat2zyx(req):
    """
    4b) This function convert a quaternion q = [q.w q.x q.y q.z]^T 
        to an euler angle representation alpha, beta and gamma for:
        R = Rz(gamma)*Ry(beta)*Rx(alpha)
          = [cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)]
            [cos(beta)*sin(gamma), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha)]
            [          -sin(beta),                                    cos(beta)*sin(alpha),                                    cos(alpha)*cos(beta)]
    """
    #q   = Quaternion()
    res = quat2zyxResponse()

    # I choose entries r_1,0, r_2,0, r_2,1 to calculate alpha, beta and gamma:
    # r_2,0: -sin(beta) = 2qxqz - 2qyqw
    beta  = np.arcsin(-(2*req.q.x*req.q.z - 2*req.q.y*req.q.w))
    res.y.data = beta

    # r_2,1: cos(beta)*sin(alpha) = 2qyqz + 2qxqw
    alpha = np.arcsin((2*req.q.y*req.q.z + 2*req.q.x*req.q.w)/np.cos(beta))
    res.x.data = alpha

    # r_1,0: cos(beta)*sin(gamma) = 2qxqy + 2qzqw
    gamma = np.arcsin((2*req.q.x*req.q.y + 2*req.q.z*req.q.w)/np.cos(beta))
    res.z.data = gamma

    return res


def convert_quat2rodrigues(req):
    """
    4c) This function convert a quaternion q = [q.w q.x q.y q.z]^T
        to rodrigues representation
    """
    res = quat2rodriguesResponse()
    # cos(theta/2) = qw
    theta = np.arccos(req.q.w)*2
    if theta == 0:
        print("Not Valid")
    else:
        # ux*sin(theta/2) = qx
        ux = req.q.x/np.sin(theta/2)

        # uy*sin(theta/2) = qy
        uy = req.q.y/np.sin(theta/2)

        # uz*sin(theta/2) = qz
        uz = req.q.z/np.sin(theta/2)

        # Rodrigues vector elements:
        res.x.data = np.tan(theta/2)*ux
        res.y.data = np.tan(theta/2)*uy
        res.z.data = np.tan(theta/2)*uz

    return res


def convert_rotmat2quat(req):
    """
    4d) This function convert a rotation matrix R = [r1; r2; r3] 
        to a quaternion q = [q.w q.x q.y q.z]^T
    """
    res = rotmat2quatResponse()

    theta = np.arccos(0.5*(req.r1.data[0] + req.r2.data[0] + req.r3.data[2] - 1))
    
    xr = req.r3.data[1] - req.r2.data[2]
    yr = req.r1.data[2] - req.r3.data[0]
    zr = req.r2.data[0] - req.r1.data[1]
    
    # Modified formula to compensate the situation when sin(theta) --> 0:
    x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

    res.q.w = np.cos(theta/2)
    res.q.x = x * np.sin(theta/2)
    res.q.y = y * np.sin(theta/2)
    res.q.z = z * np.sin(theta/2)

    return res


def rotation_converter():
    rospy.init_node('rotation_converter')
    ##TODO: Initialise the services
    # Quaternion to Euler Rz(gamma)Ry(beta)Rx(alpha):
    q2zyx = rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    # Quaternion to Rodrigues:
    q2rod = rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)

    # Rotation matrix to Quaternion:
    rot2q = rospy.Service('rotmat2quat', rotmat2quat, convert_rotmat2quat)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
