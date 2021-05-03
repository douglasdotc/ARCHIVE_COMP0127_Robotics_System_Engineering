#!/usr/bin/env python

import  rospy
import  tf2_ros
import  numpy                   as      np
from    math                    import  pi
from    sensor_msgs.msg         import  JointState
from    geometry_msgs.msg       import  TransformStamped, Quaternion
from    trajectory_msgs.msg     import  JointTrajectory


class youbot_kinematic(object):

    def __init__(self):

        self.dh_params = [[-0.033, pi/2, 0.145, pi],
                          [0.155, 0.0, 0.0, pi/2],
                          [0.135, 0.0, 0.0, 0.0],
                          [-0.002, pi/2, 0.0, -pi/2],
                          [0.0, pi, -0.185, pi]]

        self.joint_offset           = [170*pi/180, 65*pi/180, -146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min        = [-169*pi/180, -65*pi/180, -150*pi/180, -102.5*pi/180, -167.5*pi/180]
        self.joint_limit_max        = [169*pi/180, 90*pi/180, 146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.joint_state_sub  = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=5)
        self.traj_publisher   = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def dh_matrix_standard(self, a, alpha, d, theta):
        A = np.zeros((4, 4))

        A[0, 0] = np.cos(theta)
        A[0, 1] = -np.sin(theta) * np.cos(alpha)
        A[0, 2] = np.sin(theta) * np.sin(alpha)
        A[0, 3] = a * np.cos(theta)

        A[1, 0] = np.sin(theta)
        A[1, 1] = np.cos(theta) * np.cos(alpha)
        A[1, 2] = -np.cos(theta) * np.sin(alpha)
        A[1, 3] = a * np.sin(theta)

        A[2, 1] = np.sin(alpha)
        A[2, 2] = np.cos(alpha)
        A[2, 3] = d

        A[3, 3] = 1.0

        return A

    def joint_state_callback(self, msg):
        for i in range(0, 5):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine_offset(self.current_joint_position, 5)
        self.broadcast_pose(current_pose)


    def forward_kine(self, joint, frame):
        T = np.identity(4)

        for i in range(0, frame):
            A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], joint[i] + self.dh_params[i][3])

            T = T.dot(A)

        return T


    def forward_kine_offset(self, joint, frame):
        T = np.identity(4)

        for i in range(0, frame):
            if (i == 0):
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], self.joint_offset[i] - (joint[i] + self.dh_params[i][3]))
            else:
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], (joint[i] + self.dh_params[i][3]) - self.joint_offset[i])

            T = T.dot(A)

        return T


    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp    = rospy.Time.now()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id  = 'arm_end_effector'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation      = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def rotmat2q(self, T):
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

        q.w = np.cos(angle / 2)
        q.x = x * np.sin(angle / 2)
        q.y = y * np.sin(angle / 2)
        q.z = z * np.sin(angle / 2)

        return q

    def convert_quat2rodrigues(self, q):
        # cos(theta/2) = qw
        theta = np.arccos(q.w)*2
        if theta == 0:
            print("Not Valid")
        else:
            # ux*sin(theta/2) = qx
            ux = q.x/np.sin(theta/2)

            # uy*sin(theta/2) = qy
            uy = q.y/np.sin(theta/2)

            # uz*sin(theta/2) = qz
            uz = q.z/np.sin(theta/2)

            # Rodrigues vector elements:
            rex = theta*ux
            rey = theta*uy
            rez = theta*uz

        return [rex, rey, rez]

    def get_jacobian(self, joint):
        ##TODO: Fill in this function to complete the question 4a
        J     = np.zeros((6,5)) # Initialize Jacobian
        T_All = []              # Memory for transformation matrices

        # Forward KE to calculate Transformation matrices
        for frame in range(0,5):
            T_All.append(self.forward_kine(joint, frame))
        
        # p_e^0:
        p_e = T_All[len(T_All)-1][0:3,3]

        for i in range(0,5):
            # Select i th transformation matrix:
            Tr      = T_All[i]
            z_previ = Tr[0:3,2]
            o_previ = Tr[0:3,3]

            # J_Pi:
            J[0:3,i] = np.cross(z_previ, (p_e - o_previ))

            # J_Oi:
            J[3:6,i] = z_previ

        return J

    def inverse_kine_closed_form(self, desired_pose):
        ##Ignore this.
        raise NotImplementedError() #Remove this line, once implemented everything

    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Fill in this function to complete the question 4c
        
        # Assume desire_pose is a 6x1 vector (x_e^*)
        # Assume current_joint is a 5x1 vector q(0)

        # Cast input to np arrays
        desired_pose  = np.array(desired_pose)
        current_joint = np.array(current_joint)

        # Initialize:
        k           = 0
        T_e         = np.identity(4)
        x_e         = np.zeros((6,1))
        x_e_star    = np.zeros((6,1))
        alpha       = 0.1 # Converge rate
        error   = 100000

        # Error thershold:
        epsilon = 0.001
        
        while error > epsilon:
            # Forward KE for T(q(k))
            T_e = self.forward_kine(current_joint, 5)

            # Jacobian for q(k)
            J = self.get_jacobian(current_joint)

            # Pose at step k
            p_e = T_e[0:3,3]
            R_e = T_e[0:3,0:3]

            # Convert rotation matrix to rodrigues for x_e:
            [rex, rey, rez] = self.convert_quat2rodrigues(self.rotmat2q(R_e))
            
            # Current x_e
            x_e[0:3] = p_e.reshape(3,1)
            x_e[3]   = rex
            x_e[4]   = rey
            x_e[5]   = rez

            x_e_star = desired_pose

            # Pose information at q_k+1 (q_kp1)
            current_joint = current_joint + np.squeeze(alpha*np.transpose(J).dot((x_e_star - x_e)))
            # Check error
            error = np.linalg.norm(x_e_star - x_e)

            # Debug
            print("Error: {}".format(error))

            # Next step
            k = k + 1
        print("Done in iteration: {}".format(k))
        return current_joint

    def check_singularity(self, current_joint):
        ##TODO: Fill in this function to complete the question 4d
        J = self.get_jacobian(current_joint)
        
        if J.shape[0] == 6 and J.shape[1] == 6:
            det_J = np.linalg.det(J)
        elif J.shape[0] == 6 and J.shape[1] != 6:
            det_J = np.linalg.det(J.T.dot(J))

        if det_J == 0:
            print("Singular!")