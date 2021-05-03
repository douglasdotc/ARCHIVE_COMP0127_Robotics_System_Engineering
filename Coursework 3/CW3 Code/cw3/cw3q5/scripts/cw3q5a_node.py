#!/usr/bin/env python
# -*- coding: utf-8 -*-

import  rospy
import  rosbag
import  rospkg
import  sys
import  PyKDL
import  matplotlib.pyplot                   as     plt
import  numpy                               as     np
from    math                                import pi
from    sensor_msgs.msg                     import JointState
from    trajectory_msgs.msg                 import JointTrajectory, JointTrajectoryPoint
from    geometry_msgs.msg                   import Point
from    cw3q2.iiwa14Kine                    import iiwa14_kinematic
from    kdl_kine.kdl_kine_solver_cw3q5      import robot_kinematic_mine

"""
    Write a script in cw3q5a.* to use dynamic components to compute the joint accelerations 
    throughout the pre-defined trajectory defined in cw3 launch/bags/cw3bag1.bag.

    Plot the computed accelerations as part of the report. The marking criteria for this question is given below. 
    Hint: Once you run the trajectory, you can obtain the torque from the field ”effort” 
    in the JointState message.

    1. The computed acceleration is somewhat similar to the trajectory.     [4 marks]
    2. The explanation in the report on how you calculate the acceleration. [4 marks]
    3. Is this problem forward dynamics or inverse dynamics?                [2 marks]

    ##TODO: Fill in the code. You will have to add your node in the launch file as well. 
    # The launch file is in cw3_launch.
"""

def traj(my_iiwa, bag, joint_traj):
    joint_traj.header.stamp = rospy.Time.now()

    for topic, msg, t in bag.read_messages(topics=['/iiwa/EffortJointInterface_trajectory_controller/command']):
        
        for joint_name in msg.joint_names:
            joint_traj.joint_names.append(joint_name)
        
        for point in msg.points:
            my_pt = JointTrajectoryPoint()
            for i in range(7):
                my_pt.positions.append(point.positions[i])
                my_pt.velocities.append(point.velocities[i])
                my_pt.accelerations.append(point.accelerations[i])
                my_pt.time_from_start.secs = point.time_from_start.secs
            joint_traj.points.append(my_pt)
    bag.close()
    print(joint_traj)
    return joint_traj

def acceleration(my_iiwa, KDL):
    # Get torque velocity and position:
    torque = np.array(my_iiwa.current_joint_torque).reshape(7,1)
    q_dot  = np.array(my_iiwa.current_joint_velocity)
    q      = np.array(my_iiwa.current_joint_position)

    #print("q: {}".format(q))
    # 1. Calculate B
    B_KDL = KDL.getB(q)
    B = np.zeros((7,7))
    for i in range(7):
        for j in range(7):
            B[i,j] = B_KDL[i,j]
    
    # 2. Calculate C
    C_KDL = KDL.getC(q, q_dot)
    C_and_q = np.zeros((7,1))
    for i in range(7):
        C_and_q[i] = C_KDL[i]

    # 3. Calculate G
    G_KDL = KDL.getG(q)
    G = np.zeros((7,1))
    for i in range(7):
        G[i] = G_KDL[i]
    
    # 5. Calculate q_ddot
    q_ddot = np.matmul(np.linalg.inv(B), (torque - C_and_q - G))
    
    return q_ddot, q_dot


if __name__ == '__main__':
    try:
        rospy.sleep(10)
        # Initialize:
        rospy.init_node('iiwa_traj_cw3', anonymous=True)
        my_iiwa     = iiwa14_kinematic()
        rospack     = rospkg.RosPack()
        KDL         = robot_kinematic_mine('iiwa_link_0', 'iiwa_link_ee')
        
        # Paths
        path        = rospack.get_path('cw3_launch')
        bag         = rosbag.Bag(path + '/bags/cw3bag1.bag')

        # Trajectory:
        joint_traj  = JointTrajectory()
        traj_pub    = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
        joint_traj  = traj(my_iiwa, bag, joint_traj)
        rospy.sleep(1)
        traj_pub.publish(joint_traj)
        rospy.sleep(1)

        q_ddot          = []
        ground_accel    = []
        q_dot_prev      = np.zeros((7,1))
        t = 0
        while not rospy.is_shutdown():
            if t <= 300:
                q_ddot_i, q_dot_i = acceleration(my_iiwa, KDL)
                ground_accel_i = np.divide((q_dot_i.reshape(7,1) - q_dot_prev), 0.1)
                q_dot_prev     = np.copy(q_dot_i.reshape(7,1))
                t             += 1
                q_ddot.append(np.squeeze(q_ddot_i.T).tolist())
                ground_accel.append(np.squeeze(ground_accel_i.T).tolist())
                rospy.sleep(0.1)
            else:
                break
        
        q_ddot = np.array(q_ddot)
        ground_accel = np.array(ground_accel)

        a_file = open("/home/douglasc/catkin_ws/src/comp0127_lab/cw3/cw3q5/scripts/q_ddot.txt", "w")
        np.savetxt(a_file, q_ddot, fmt='%f', delimiter=',')
        a_file.close()
        
        a_file = open("/home/douglasc/catkin_ws/src/comp0127_lab/cw3/cw3q5/scripts/ground_accel.txt", "w")
        np.savetxt(a_file, ground_accel, fmt='%f', delimiter=',')
        a_file.close()

    except rospy.ROSInterruptException:
        pass