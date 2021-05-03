#!/usr/bin/env python

import  rospy
import  tf2_ros
import  numpy               as     np
from    math                import pi
from    sensor_msgs.msg     import JointState
from    geometry_msgs.msg   import TransformStamped, Quaternion

class iiwa14_kinematic(object):

    def __init__(self):
        ##TODO: Fill in the DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro)
        ##                          a    alpha  d       theta
        self.DH_params = np.array([[0.0, -pi/2, 0.2025, 0.0],
                                   [0.0,  pi/2, 0.0000, 0.0],
                                   [0.0,  pi/2, 0.4200, 0.0],
                                   [0.0, -pi/2, 0.0000, 0.0],
                                   [0.0, -pi/2, 0.4000, 0.0],
                                   [0.0,  pi/2, 0.0000, 0.0],
                                   [0.0,   0.0, 0.1260, 0.0]])

        self.current_joint_position     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocity     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_torque       = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]

        ##The mass of each link.
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]

        self.link_COM_pos = np.array([
            [      0,    -0.03,          0.1200],
            [-0.0003,   -0.059,          0.0420],
            [      0,     0.03,   0.13 + 0.2045],
            [      0,    0.067,          0.0340],
            [-0.0001,   -0.021,  0.076 + 0.1845],
            [      0,  -0.0006,          0.0004],
            [      0,        0,   0.02 + 0.0810]
        ])
        # self.link_COM_pos = np.array([
        #     [     0,    -0.03,   0.12],
        #     [0.0003,    0.059,  0.042],
        #     [     0,     0.03,   0.13],
        #     [     0,    0.067,  0.034],
        #     [0.0001,    0.021,  0.076],
        #     [     0,   0.0006, 0.0004],
        #     [     0,        0,   0.02]
        # ])

        ##Moment on inertia of each link, defined at the centre of mass.
        ##Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        self.Ixyz = np.array([
            [0.1,     0.09,   0.02],
            [0.05,    0.018,  0.044],
            [0.08,    0.075,  0.01],
            [0.03,    0.01,   0.029],
            [0.02,    0.018,  0.005],
            [0.005,   0.0036, 0.0047],
            [0.001,   0.001,  0.001]
        ])

        ##gravity
        self.g = 9.8

        # Q5a
        # self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback,
        #                                         queue_size=5)
        # Q5b
        self.joint_state_sub = rospy.Subscriber('/object_iiwa/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)

        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        for i in range(0, 7):
            self.current_joint_position[i] = msg.position[i]
            self.current_joint_velocity[i] = msg.velocity[i]
            self.current_joint_torque[i]   = msg.effort[i]
            
        current_pose = self.forward_kine(self.current_joint_position, 7)
        self.broadcast_pose(current_pose)

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

    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()

        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

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
            return [0,0,0]
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

    ##Useful Transformation function
    def T_translation(self, t):
        T = np.identity(4)
        for i in range(0, 3):
            T[i, 3] = t[i]
        return T

    ##Useful Transformation function
    def T_rotationZ(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationX(self, theta):
        T = np.identity(4)
        T[1, 1] = np.cos(theta)
        T[1, 2] = -np.sin(theta)
        T[2, 1] = np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationY(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 2] = np.sin(theta)
        T[2, 0] = -np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    def broadcast_joint(self, transform, pose, frame_idx):
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'test_iiwa_link_{}'.format(frame_idx)

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def forward_kine(self, joint, frame):
        T = np.identity(4)
        ##Add offset from the iiwa platform.
        T[2, 3] = 0.1575
        ##TODO: Fill in this function to complete Q2.

        transform = TransformStamped()
        for i in range(0, frame):
            A = self.dh_matrix_standard(self.DH_params[i][0], self.DH_params[i][1], self.DH_params[i][2], joint[i] + self.DH_params[i][3])
            T = T.dot(A)
            self.broadcast_joint(transform, T, i+11)
        return T

    def forward_kine_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to the centre of mass of the specified link.
        transform = TransformStamped()
        
        prev_frame       = frame - 1
        T_0_prev         = self.forward_kine(joint, prev_frame)
        Rot_theta_i      = self.T_rotationZ(joint[prev_frame] + self.DH_params[prev_frame, 3])
        Trans_G_i        = np.eye(4)
        Trans_G_i[0:3,3] = self.link_COM_pos[prev_frame,:].reshape(3,)
        T                = T_0_prev.dot(Rot_theta_i).dot(Trans_G_i)
        self.broadcast_joint(transform, T, frame)
        return T

    def get_jacobian(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at each frame.
        
        J     = np.zeros((6,7)) # Initialize Jacobian
        T_All = []              # Memory for transformation matrices

        # Forward KE to calculate Transformation matrices
        for frame in range(0,7):
            T_All.append(self.forward_kine(joint, frame))
        
        # p_e^0:
        p_e = T_All[len(T_All)-1][0:3,3]

        for i in range(0,7):
            # Select i th transformation matrix:
            Tr      = T_All[i]
            z_previ = Tr[0:3,2]
            o_previ = Tr[0:3,3]

            # J_Pi:
            J[0:3,i] = np.cross(z_previ, (p_e - o_previ))

            # J_Oi:
            J[3:6,i] = z_previ

        return J

    def get_jacobian_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the centre of mass of the specified link.
        
        J      = np.zeros((6,7)) # Initialize Jacobian
        T_All  = []              # Memory for transformation matrices
        T_0_Gi = self.forward_kine_cm(joint, frame)
        p_li   = T_0_Gi[0:3,3]

        # Forward KE to calculate Transformation matrices
        for frame_idx in range(0,frame):
            T_All.append(self.forward_kine(joint, frame_idx))

        for i in range(0,frame):
            # Select i th transformation matrix:
            Tr      = T_All[i]
            z_previ = Tr[0:3,2]
            p_previ = Tr[0:3,3]

            # J_Pi:
            J[0:3,i] = np.cross(z_previ, (p_li - p_previ))

            # J_Oi:
            J[3:6,i] = z_previ
        #print(J)
        #print(frame)
        return J

    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## "current_joint" is an array of double consisting of the joint value (works as a starting point for the optimisation).
        ## The output is numpy vector containing an optimised joint configuration for the desired pose.

        # Initialize:
        k           = 0
        T_e         = np.identity(4)
        x_e         = np.zeros((6,1))
        x_e_star    = np.zeros((6,1))
        error       = 100000
        alpha       = 0.1               # Converge rate
        epsilon     = 0.001             # Error thershold

        # Convert desired_pose (4x4 np matrix) to a 6x1 vector (x_e^*):
        # Pose at step k
        p_e_desired = desired_pose[0:3,3]
        R_e_desired = desired_pose[0:3,0:3]

        # Convert rotation matrix to rodrigues for x_e_star:
        [rex_desired, rey_desired, rez_desired] = self.convert_quat2rodrigues(self.rotmat2q(R_e_desired))
        
        # x_e_star:
        x_e_star[0:3] = p_e_desired.reshape(3,1)
        x_e_star[3]   = rex_desired
        x_e_star[4]   = rey_desired
        x_e_star[5]   = rez_desired
        
        while error > epsilon:
            # Forward KE for T(q(k))
            T_e = self.forward_kine(current_joint, 7)

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

            # x_e_star = desired_pose

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

    def inverse_kine_closed_form(self, desired_pose):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## The output is a numpy matrix consisting of the joint value for the desired pose.
        ## You may need to re-structure the input of this function.
        raise NotImplementedError() #Remove this line, once implemented everything

    def getB(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 7*7 matrix.

        # Initialize:
        B = np.zeros((7,7))

        for frame_idx in range(1,8):
            # 1. Forward KE at COM --> Get T --> R
            R_0_Gi         = self.forward_kine_cm(joint, frame_idx)[0:3, 0:3]
            
            # 2. Inertia matrix
            I_li_Oi_p      = np.eye(3)
            I_li_Oi_p[0,0] = self.Ixyz[frame_idx - 1, 0]
            I_li_Oi_p[1,1] = self.Ixyz[frame_idx - 1, 1]
            I_li_Oi_p[2,2] = self.Ixyz[frame_idx - 1, 2]
            
            # 3. I_CAL_li
            I_CAL_li       = np.matmul(np.matmul(R_0_Gi, I_li_Oi_p), R_0_Gi.T)

            # 4. Jacobian at COM
            J_li           = self.get_jacobian_cm(joint, frame_idx)
            J_P_li         = J_li[0:3,:]
            J_O_li         = J_li[3:6,:]

            # 5. mass
            m_li           = self.mass[frame_idx - 1]

            # 6. Combine:
            B             += m_li*np.matmul(J_P_li.T, J_P_li) + np.matmul(np.matmul(J_O_li.T, I_CAL_li), J_O_li)
            #print(B)
            #print(frame_idx)
        assert B.shape == (7,7)
        return B

    def getC(self, joint, vel):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.  q
        ## "vel" is a numpy array of double consisting of the joint velocity. q_dot
        ## The output is a numpy 7*7 matrix.
        
        # Initialize:
        C     = np.zeros((7,7))
        eps   = 0.000001
        
        for i in range(0,7):
            for j in range(0,7):
                for k in range(0,7):
                    B           = self.getB(joint)
                    joints_k    = np.copy(joint)
                    joints_k[k] = joints_k[k] + eps
                    B_ij        = self.getB(joints_k)
                    joint_i     = np.copy(joint)
                    joint_i[i]  = joint_i[i] + eps
                    B_jk        = self.getB(joint_i)
                    x           = (B_ij[i,j]/eps) - (B[i,j]/eps)
                    y           = 0.5*((B_jk[j,k]/eps) - (B[j,k]/eps))
                    C[i,j]     += (x - y)*vel[k]

        assert C.shape == (7,7)
        return C
    
    def getG(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy array 7*1.
        G   = np.zeros((7,1))
        g_0 = np.array([0, 0, -self.g]).reshape(3,1)
        eps = 0.000001

        for i in range(0,7):
            P_q = 0.0
            P_q_eps = 0.0
            for j in range(0,7):
                m_lj        = self.mass[j]
                joint_i     = np.copy(joint)
                joint_i[i]  = joint[i] + eps
                p_lj        = self.forward_kine_cm(joint, j + 1)[0:3,3]
                p_li_eps    = self.forward_kine_cm(joint_i, j + 1)[0:3,3]
                
                # Sum:
                P_q      -= m_lj*g_0.T.dot(p_lj)
                P_q_eps  -= m_lj*g_0.T.dot(p_li_eps)
            G[i] = (P_q_eps - P_q)/eps
        assert G.shape == (7,1)
        return G