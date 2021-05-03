#!/usr/bin/env python

import  rospy
import  numpy                       as      np
from    math                        import  pi
from    youbotKine                  import  youbot_kinematic
from    kdl_kine.kdl_kine_solver    import  robot_kinematic

YB_CAL = youbot_kinematic()
rospy.init_node('cw2q4_node')
rate   = rospy.Rate(10)

def check_jacobian():
    current_joint_values = []
    for joint in range(5):
        current_joint_values.append(np.random.uniform(low=YB_CAL.joint_limit_min[joint], high=YB_CAL.joint_limit_max[joint]))
    
    current_joint_position = list(current_joint_values)

    h_kine = robot_kinematic('arm_link_0', 'arm_link_ee')

    while not rospy.is_shutdown():
        print('Jacobian from KDL: ')
        print(h_kine.get_jacobian(h_kine.current_joint_position))
        rate.sleep()
        
        J = YB_CAL.get_jacobian(current_joint_position)

        print('Jacobian from calculation: ')
        print(np.round(J, 6))

def check_ite_IKE(desired_pose):
    current_joint_values = []
    for joint in range(5):
        current_joint_values.append(np.random.uniform(low=YB_CAL.joint_limit_min[joint], high=YB_CAL.joint_limit_max[joint]))
    
    current_joint_position = list(current_joint_values)

    return YB_CAL.inverse_kine_ite(desired_pose, current_joint_position)

if __name__ == "__main__":
    Check = "ITE"
    
    if Check == "J":
        # Test Jacobian
        check_jacobian()

    elif Check == "ITE":
        # Test ITE IKE
        desired_joint_position = []
        desired_pose = np.zeros((6,1))
        
        for joint in range(5):
            desired_joint_position.append(np.random.uniform(low=YB_CAL.joint_limit_min[joint], high=YB_CAL.joint_limit_max[joint]))
        
        print("Desired pose: {}".format(desired_joint_position))
        T_e = YB_CAL.forward_kine(desired_joint_position, 5)
        p_e = T_e[0:3,3]
        R_e = T_e[0:3,0:3]

        [rex, rey, rez] = YB_CAL.convert_quat2rodrigues(YB_CAL.rotmat2q(R_e))

        desired_pose[0:3] = p_e.reshape(3,1)
        desired_pose[3]   = rex
        desired_pose[4]   = rey
        desired_pose[5]   = rez

        Soln = check_ite_IKE(desired_pose)
        print(Soln)
    
