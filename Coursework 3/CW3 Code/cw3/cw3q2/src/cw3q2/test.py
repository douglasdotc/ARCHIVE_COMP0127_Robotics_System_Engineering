#!/usr/bin/env python

import rospy
import random
from math                     import  pi
from sensor_msgs.msg          import  JointState
from kdl_kine.kdl_kine_solver import  robot_kinematic
from kdl_kine.urdf            import*
import numpy                  as      np
from iiwa14Kine               import  iiwa14_kinematic

def show_jacobian():
    print("Attach Debugger!")
    rospy.sleep(3)
    rospy.init_node('kuka_dynamics_node')

    rate = rospy.Rate(10)
    # Initiate the base link(where the kinematic chain starts) and the end-effector link(where it ends).
    # Please look at the urdf file of the respective robot arm for the names.
    iiwa_kine = robot_kinematic('iiwa_link_0', 'iiwa_link_ee')
    test_iiwa_kine = iiwa14_kinematic()

    q = PyKDL.JntArray(7)
    qdot = PyKDL.JntArray(7)

    while not rospy.is_shutdown():
        #Because joint_state_publisher does not publish velocity, we manually generate random joint positions and velocities as input to B, C, g matrices.
        # for i in range(7):
        #     q[i] = random.uniform(-1, 1)
        #     qdot[i] = random.uniform(-0.5, 0.5)
        q_current = [0,0,0,0,0,0,0]
        q    = [pi/4, -7*pi/36, 11*pi/36,  -pi/6, -5*pi/36, 13*pi/36,   -pi/18]
        qdot = [-0.2, 0.1, 0.1, 0.1, 0.3, 0.2,0.1]

        print('q_current')
        desired_T = test_iiwa_kine.forward_kine(q,7)
        current_joint = test_iiwa_kine.inverse_kine_ite(desired_T, q_current)
        print(current_joint)
        # sub  = rospy.Subscriber('/joint_states', JointState, test_iiwa_kine.forward_kine(q, 7))
        # rate = rospy.Rate(10)
        # rospy.spin()
        # rate.sleep()
        # print(iiwa_kine.forward_kinematics(q))
        print('B(q): ')
        #PyKDL has a unique representation for the matrix B (only in Python, doesn't apply to C++), so we have to convert the representation to a matrix manually.
        B = iiwa_kine.getB(q)

        Bmat = np.zeros((7, 7))
        for i in range(7):
            for j in range(7):
                Bmat[i, j] = B[i, j]
        print('B_diff: ')
        print(test_iiwa_kine.getB(q))
        print(Bmat)

        print('C(q, qdot) diff: ')
        Cq = iiwa_kine.getC(q, qdot)
        Cmat = np.zeros((7,1))
        for i in range(7):
            Cmat[i] = Cq[i]
        print(test_iiwa_kine.getC(q, qdot).dot(np.array(qdot).reshape(7,1)))
        print(Cmat)

        print('g(q) diff: ')
        G = iiwa_kine.getG(q)
        Gmat = np.zeros((7,1))
        for i in range(7):
            Gmat[i] = G[i]
        print(test_iiwa_kine.getG(q))
        print(Gmat)

        rate.sleep()


if __name__ == '__main__':
    show_jacobian()
