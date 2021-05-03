#!/usr/bin/env python
# -*- coding: utf-8 -*-

from    __future__                          import print_function
from    math                                import pi
from    sympy                               import *
from    sensor_msgs.msg                     import JointState
from    trajectory_msgs.msg                 import JointTrajectory, JointTrajectoryPoint
from    geometry_msgs.msg                   import Point
from    cw3q2.iiwa14Kine                    import iiwa14_kinematic
from    kdl_kine.kdl_kine_solver_cw3q5      import robot_kinematic_mine
import  matplotlib.pyplot                   as     plt
import  numpy                               as     np
import  rospy
import  rosbag
import  rospkg
import  sys
import  PyKDL

"""
    Write a script in cw3q5b.* to determine the mass and the centre of mass of an
    object attached to the KUKA iiwa robot (with respected to the last frame as defined
    by the given parameters). The mass and centre of mass should be output to the ROS
    info log when the submitted launch file is executed. The output should contain the
    calculated masses and centres of masses for each configuration and the average
    among the configurations. The marking criteria of this question is shown below.

    q_a = [45◦, −35◦, 55◦, −30◦, −25◦, 65◦, −10◦]^T
    q_b = [5◦, 40◦, 80◦, 10◦, 8◦, −50◦, −10◦]^T
    q_c = [−50◦, 60◦, 0◦, −15◦, 60◦, −25◦, −50◦]^T

    1. The calculated mass has less than 5 percents error.                              [4 marks]
    2. The calculated centre of mass has less than 7 percents error                     [4 marks]

    3. Calculate the external torque exerting on each joint when the joint position 
        
        q_d = [0◦, −90◦, −90◦, 90◦, −90◦, 90◦, −30◦]^T                                  [4 marks]

    4. The explanation in the report on how you calculate these values (3 marks for
    the mass, 2 for the centre of mass and 3 for the external torque).                  [8 marks]

    ##TODO: Fill in the code. You will have to add your node in the launch file as well.
    # The launch file is in cw3_launch.
"""

def pose(joint_traj, q_choice = 'a'):
    
    iiwa_joint_names = ['object_iiwa_joint_1', 
                        'object_iiwa_joint_2', 
                        'object_iiwa_joint_3', 
                        'object_iiwa_joint_4', 
                        'object_iiwa_joint_5', 
                        'object_iiwa_joint_6', 
                        'object_iiwa_joint_7']
                        
    if q_choice == 'a':
        q = [    pi/4, -7*pi/36, 11*pi/36,  -pi/6, -5*pi/36, 13*pi/36,   -pi/18]

    elif q_choice == 'b':
        q = [   pi/36,   2*pi/9,   4*pi/9,  pi/18,  2*pi/45, -5*pi/18,   -pi/18]

    elif q_choice == 'c':
        q = [-5*pi/18,     pi/3,      0.0, -pi/12,     pi/3, -5*pi/36, -5*pi/18]

    elif q_choice == 'd':
        q = [     0.0,    -pi/2,    -pi/2,   pi/2,    -pi/2,     pi/2,    -pi/6]

    joint_traj.header.stamp = rospy.Time.now()
    for joint_name in iiwa_joint_names:
        joint_traj.joint_names.append(joint_name)

    my_pt = JointTrajectoryPoint()
    for i in range(7):
        my_pt.positions.append(q[i])
        my_pt.velocities.append(0)
        my_pt.accelerations.append(0)
        my_pt.time_from_start.secs = 10
    joint_traj.points.append(my_pt)
    
    return joint_traj

def torque_external(KDL, torque, q_dot, q):

    G_KDL = KDL.getG(q)
    G = np.zeros((7,1))
    for i in range(7):
        G[i] = G_KDL[i]
        
    torque_ext = torque - G

    return torque_ext

def joint_positions(my_iiwa, q):
    # Get positions of joints
    
    joints_pos = []
    joints_z   = []
    for frame_idx in range(7):
        T = my_iiwa.forward_kine(q, frame_idx)
        joints_pos.append(T[0:3,3])
        joints_z.append(T[0:3,2])
    
    return joints_pos, joints_z

def mass_COM(torque_ext, selected_joints_pos, selected_joints_z):
    g = 9.8
    A = np.array([
        [torque_ext[0][0]/g, - selected_joints_z[0][0], selected_joints_z[0][1]],
        [torque_ext[1][0]/g, - selected_joints_z[1][0], selected_joints_z[1][1]],
        [torque_ext[2][0]/g, - selected_joints_z[2][0], selected_joints_z[2][1]]
    ])
    B = np.array([
        [selected_joints_z[0][1]*selected_joints_pos[0][0] - selected_joints_z[0][0]*selected_joints_pos[0][1]],
        [selected_joints_z[1][1]*selected_joints_pos[1][0] - selected_joints_z[1][0]*selected_joints_pos[1][1]],
        [selected_joints_z[2][1]*selected_joints_pos[2][0] - selected_joints_z[2][0]*selected_joints_pos[2][1]]
    ])
    soln = np.linalg.solve(A, B)
    m = 1/soln[0][0]
    y = soln[1][0]
    x = soln[2][0]
    z = (selected_joints_pos[2][2]*np.sqrt(x**2 + y**2))/np.sqrt(selected_joints_pos[2][0]**2 + selected_joints_pos[2][1]**2)
    COM = [x, y, z]
    return m, COM

if __name__ == '__main__':
    try:
        rospy.sleep(10)
        # Initialize:
        rospy.init_node('iiwa_traj_cw3', anonymous=True)
        my_iiwa     = iiwa14_kinematic()
        KDL         = robot_kinematic_mine('object_iiwa_link_0', 'object_iiwa_link_ee')
        q_choice    = 'd'
        
        # Trajectory:
        joint_traj  = JointTrajectory()
        traj_pub    = rospy.Publisher('/object_iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
        joint_traj  = pose(joint_traj, q_choice)
        rospy.sleep(1)
        traj_pub.publish(joint_traj)
        rospy.sleep(1)

        choice = [2,4,6]
        while not rospy.is_shutdown():
            # Get torque velocity and position:
            torque = np.array(my_iiwa.current_joint_torque).reshape(7,1)
            q_dot  = np.array(my_iiwa.current_joint_velocity)
            q      = np.array(my_iiwa.current_joint_position)

            torque_ext           = torque_external(KDL, torque, q_dot, q)
            joints_pos, joints_z = joint_positions(my_iiwa, q)

            if q_choice != 'd':
                selected_torque_ext  = torque_ext[choice]
                selected_joints_pos  = np.array(joints_pos)[choice]
                selected_joints_z    = np.array(joints_z)[choice]
                m, COM               = mass_COM(selected_torque_ext, selected_joints_pos, selected_joints_z)
                print('m: {},\t COM: {}'.format(m, COM))
            else:
                print('External Torques of joint 1 - 7:\n{}'.format(torque_ext))
    except rospy.ROSInterruptException:
        pass