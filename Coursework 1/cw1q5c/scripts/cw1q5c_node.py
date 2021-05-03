#!/usr/bin/env python
###################################
# COMP0127 CW1
# Author:   Douglas Chiang Hou Nam
# SN:       15055142
###################################
import rospy
import tf2_ros
import numpy            as np
from math               import pi
from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import TransformStamped, Quaternion

#Initialise all DH parameters
a       = [0.0,             0.0,    0.155,  0.135,  0.0,    0.0]
alpha   = [0.0,             -pi/2,  0.0,    0.0,    pi/2,   0.0]
theta   = [0.0,             -pi/2,  0.0,    pi/2,   0.0,    0.0]
d       = [0.072 + 0.075,   0.0,    0.0,    0.0,    0.113,  0.105]

name_link = ['arm5c_link_1', 'arm5c_link_2', 'arm5c_link_3', 'arm5c_link_4', 'arm5c_link_5', 'arm5c_link_6']

# Broadcaster
br = tf2_ros.TransformBroadcaster()

def fkine_Modified(a, alpha, d, theta):
#Define a transformation in standard DH.
    A = np.zeros((4, 4))

    A[0, 0] = np.cos(theta)
    A[0, 1] = -np.sin(theta)
    A[0, 3] = a

    A[1, 0] = np.sin(theta)*np.cos(alpha)
    A[1, 1] = np.cos(theta)*np.cos(alpha)
    A[1, 2] = -np.sin(alpha)
    A[1, 3] = -d*np.sin(alpha)

    A[2, 0] = np.sin(theta)*np.sin(alpha)
    A[2, 1] = np.cos(theta)*np.sin(alpha)
    A[2, 2] = np.cos(alpha)
    A[2, 3] = d*np.cos(alpha)

    A[3, 3] = 1.0
    
    return A


def rotmat2q(T): 
#Define a function  for converting rotation matrix to a quaternion
    q = Quaternion()

    angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2)

    xr = T[2, 1] - T[1, 2]
    yr = T[0, 2] - T[2, 0]
    zr = T[1, 0] - T[0, 1]

    x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

    q.w = np.cos(angle/2)
    q.x = x * np.sin(angle/2)
    q.y = y * np.sin(angle/2)
    q.z = z * np.sin(angle/2)

    return q

##TODO: Complete the forward kinematic routine. The function should compute the transformation between each frame and use tf broadcaster to publish the transformation.
def forward_kinematic(joint_msg):
    T = np.identity(4)

    transform = TransformStamped()
    for i in range(6):

        A = fkine_Modified(a[i], alpha[i], d[i], theta[i] - joint_msg.position[i])
        
        transform.header.stamp      = rospy.Time.now()
        transform.header.frame_id   = 'base_link'
        transform.child_frame_id    = name_link[i]

        T = T.dot(A)
        
        transform.transform.translation.x   = T[0, 3]
        transform.transform.translation.y   = T[1, 3]
        transform.transform.translation.z   = T[2, 3]
        transform.transform.rotation        = rotmat2q(T)

        br.sendTransform(transform)

def main():
    rospy.init_node('forward_kinematic_node')
    ##TODO: Initialise the subscriber to the topic "/joint_states" and its callback function forward_kinematic
    sub  = rospy.Subscriber('/joint_states', JointState, forward_kinematic)
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == "__main__":
    main()