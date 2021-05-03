#!/usr/bin/env python

import  rospy
import  rosbag
import  rospkg
import  sys
import  PyKDL
import  numpy                               as     np
from    trajectory_msgs.msg                 import JointTrajectory, JointTrajectoryPoint
from    geometry_msgs.msg                   import Point
from    cw2q4.youbotKine                    import youbot_kinematic
from    kdl_kine.kdl_kine_solver_cw2q6      import robot_kinematic_mine
from    potential_field                     import PotentialField

joint_lower_limits = [0, 0, -np.pi*(146+151)/180, 0, 0]
joint_upper_limits = [np.pi*(169+169)/180, np.pi*(90+65)/180, 0, np.pi*(102.5+102.5)/180, np.pi*(167.5+167.5)/180]

##TODO: You can design your code to achieve the q6 task however you want as long as it stays in this file and it runs.
class Greedy(object):
    def __init__(self, cost_matrix, rank):
        self.cost_matrix = cost_matrix
        self.length      = rank

    def greedy_algorithm(self): 
        unvisited_checkpoints = list(range(2, self.length + 1))
        scheduled_checkpoints = [1]
        total_distance        = 0

        while len(unvisited_checkpoints) > 0:
            min_distance   = self.cost_matrix.max()
            min_checkpoint = 0

            for checkpoint_idx in range(len(unvisited_checkpoints)):
                current_distance = self.cost_matrix[unvisited_checkpoints[checkpoint_idx] - 1, scheduled_checkpoints[-1] - 1]
                
                if current_distance == 0 or current_distance < min_distance:
                    min_distance   = current_distance
                    min_checkpoint = checkpoint_idx

            total_distance += min_distance
            scheduled_checkpoints.append(unvisited_checkpoints[min_checkpoint])
            unvisited_checkpoints.remove(unvisited_checkpoints[min_checkpoint])

        return scheduled_checkpoints

def cost_M(checkpoints):
    cost_matrix = np.zeros((checkpoints.shape[0], checkpoints.shape[0]))
    for i in range(checkpoints.shape[0]):
        for j in range(checkpoints.shape[0]):
            cost_matrix[i,j] = np.sqrt((checkpoints[i][0] - checkpoints[j][0])**2 + (checkpoints[i][1] - checkpoints[j][1])**2 + (checkpoints[i][2] - checkpoints[j][2])**2)

    return cost_matrix

def shortest_traj(my_youbot, bag, joint_traj):
    # Return a sorted list of joint data for a shortest path
    tfs              = 5
    num_msg          = 0
    temp_joint_traj  = JointTrajectory() # unsorted trajectory
    KDL              = robot_kinematic_mine('base_link', 'arm_link_ee')
    current_position = my_youbot.current_joint_position

    joint_traj.header.stamp = rospy.Time.now()
    temp_joint_traj.header.stamp = rospy.Time.now()

    for i in range(1,6):
        joint_traj.joint_names.append('arm_joint_{}'.format(i))
        temp_joint_traj.joint_names.append('arm_joint_{}'.format(i))

    # Extract all the msg to temp_joint_traj:
    for topic, msg, t in bag.read_messages(topics=['joint_data']):
        my_pt = JointTrajectoryPoint()
        if len(msg.position) != 0:
            for i in range(0, 5):
                my_pt.positions.append(msg.position[i])
                my_pt.velocities.append(msg.position[i])
                my_pt.accelerations.append(0)
        
        temp_joint_traj.points.append(my_pt)
        num_msg += 1
    bag.close()

    # Create a n x 3 matrix of n 3D coordinates:
    checkpoints = []

    #This "joint" is PyKDL.JntArray. The output is a PyKDL.Frame. 
    initial_joint_pos = PyKDL.JntArray(5)
    for i in range(5):
        initial_joint_pos[i] = current_position[i]
    initial_pose = KDL.forward_kinematics(initial_joint_pos)
    checkpoints.append(initial_pose.p)
    # Need to replace with subscriber end point 3D coordinate:
    # T_e = my_youbot.forward_kine_offset([0.0,0.0,0.0,0.0,0.0], 5)
    # checkpoints.append(T_e[0:3,3])
    
    # Get checkpoint coordinates:
    for point in temp_joint_traj.points:
        # Get the position of the end point (p_e) for each pose using forward kinematics:
        joint_pos = PyKDL.JntArray(5)
        for i in range(5):
            joint_pos[i] = point.positions[i]
        pose = KDL.forward_kinematics(joint_pos)
        checkpoints.append(pose.p)
        #T_e = my_youbot.forward_kine_offset(point.positions, 5)
        #checkpoints.append(T_e[0:3,3])

    checkpoints = np.array(checkpoints)

    # Calculate cost matrix:
    cost_matrix = cost_M(checkpoints)

    # Shortest path:
    Greed                 = Greedy(cost_matrix, cost_matrix.shape[0])
    scheduled_checkpoints = Greed.greedy_algorithm()
    
    # Append the checkpoints in the order of the shortest path:
    # temp_joint_traj.points[scheduled_checkpoints[0] - 1].positions
    
    for i in range(1, len(scheduled_checkpoints)):
        # my_pt = JointTrajectoryPoint()

        # for j in range(0, 5):
        #     my_pt.positions.append(temp_joint_traj.points[scheduled_checkpoints[i] - 2].positions[j])

        # my_pt.time_from_start.secs = tfs
        # tfs += 60
        # joint_traj.points.append(my_pt)
        
        joint_traj, tfs = interpolate(KDL, current_position, checkpoints[scheduled_checkpoints[i-1] - 1], checkpoints[scheduled_checkpoints[i] - 1], tfs, joint_traj)

    print("joint_traj: {}".format(joint_traj))
    rospy.sleep(5)
    return joint_traj

def interpolate(KDL, current_position, point_1, point_2, tfs, joint_traj):
    # 1. Interpolate intermediate points:
    n = 5
    x = point_1[0]
    y = point_1[1]
    z = point_1[2]

    for i in range(0, n):
        my_pt = JointTrajectoryPoint()
        # 1.1 Interpolate:
        delta_x = (point_2[0] - point_1[0])/n
        delta_y = (point_2[1] - point_1[1])/n
        delta_z = (point_2[2] - point_1[2])/n
        x = x + delta_x
        y = y + delta_y
        z = z + delta_z
        position = [x,y,z]

        # 2. Inverse KE by KDL for each points to get pose:
        # Desired pose:
        pos = PyKDL.Vector(position[0], position[1], position[2])
        desired_pose = PyKDL.Frame(pos)

        # Save a list of possible pose solutions:
        selected_pose = []
        
        for i in range(1000):
            pose = KDL.inverse_kinematics_closed(desired_pose)
            selected_pose.append(pose)

        # Compare poses in selected_pose with the current_position using Mean squared error:
        mse_all = []
        for pose in selected_pose:
            for i in range(5):
                mse = (pose[i] - current_position[i])**2
            mse /= 5
            mse_all.append(mse)

        current_position = selected_pose[np.argmin(mse_all)]

        # 3. Append to joint_traj
        for j in range(0, 5):
            my_pt.positions.append(selected_pose[np.argmin(mse_all)][j])
        
        my_pt.time_from_start.secs = tfs
        tfs += 60/n
        joint_traj.points.append(my_pt)

    return joint_traj, tfs

def q2rot(q):
    rot = np.array([
        [1 - 2*q.y**2 - 2*q.z**2,   2*q.x*q.y - 2*q.z*q.w,      2*q.x*q.z + 2*q.y*q.w],
        [2*q.x*q.y + 2*q.z*q.w,     1 - 2*q.x**2 - 2*q.z**2,    2*q.y*q.z - 2*q.x*q.w],
        [2*q.x*q.z - 2*q.y*q.w,     2*q.y*q.z + 2*q.x*q.w,      1 - 2*q.x**2 - 2*q.y**2]
    ])
    return rot

def PotentialField_PathPlanning(start_position, goal_position):
    sx           = start_position[0]    # start x position
    sy           = start_position[1]    # start y positon
    sz           = start_position[2]    # start z position
    gx           = goal_position[0]     # goal x position
    gy           = goal_position[1]     # goal y position
    gz           = goal_position[2]     # goal z position 
    PF           = PotentialField()

    map_grid_size = 0.05  # potential grid resolution
    joint_radius  = 0.3   # robot radius

    # obstacle x position list
    ox = [-0.314882, -0.328049, 0.317510]  
    # obstacle y position list
    oy = [-0.161618, 0.226524, 0.06442]  
    # obstacle z position list
    oz = [0.099055, 0.041963, 0.069562]  
    
    # Calculate path:
    x, y, z = PF.potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, map_grid_size, joint_radius)
    #*** Program stuck inside the class when calculating potential map 
    # when gazebo / RViz is initializing...
    print("Path: {}, {}, {}".format(x, y, z))

    # Inverse KE for all points in the path
    # Append to joint_traj


def obstable_traj(my_youbot, bag, joint_traj):
    tfs     = 5
    num_msg = 0
    KDL     = robot_kinematic_mine('base_link', 'arm_link_ee')

    joint_traj.header.stamp = rospy.Time.now()
    for i in range(1,6):
        joint_traj.joint_names.append('arm_joint_{}'.format(i))
    
    checkpoint_frames = []
    initial_position  = my_youbot.current_joint_position
    current_position  = my_youbot.current_joint_position
    # initial_joint_pos = PyKDL.JntArray(5)
    # for i in range(5):
    #     initial_joint_pos[i] = initial_position[i]
    # initial_pose = KDL.forward_kinematics(initial_joint_pos)
    # checkpoint_frames.append(initial_pose)
    
    # Extract all the msg to checkpoint_frames:
    for topic, msg, t in bag.read_messages(topics=['target_position']):
        x = msg.translation.x
        y = msg.translation.y
        z = msg.translation.z
        rot = q2rot(msg.rotation)
        #print([x,y,z])
        pos_kdl = PyKDL.Vector(x, y, z)
        rot_kdl = PyKDL.Rotation(rot[0, 0], rot[0, 1], rot[0, 2], rot[1, 0], rot[1, 1], rot[1, 2], rot[2, 0], rot[2, 1], rot[2, 2])
        checkpoint_frame = PyKDL.Frame(rot_kdl, pos_kdl)
        checkpoint_frames.append(checkpoint_frame)
    bag.close()

    for i in range(len(checkpoint_frames)):
        selected_pose = []
        
        for k in range(1000):
            pose = KDL.inverse_kinematics_closed(checkpoint_frames[i])
            selected_pose.append(pose)

        # Compare poses in selected_pose with the current_position using Mean squared error:
        mse_all = []
        for pose in selected_pose:
            for j in range(5):
                mse = (pose[j] - current_position[j])**2
            mse /= 5
            mse_all.append(mse)

        current_position = selected_pose[np.argmin(mse_all)]

        my_pt = JointTrajectoryPoint()
        my_pt.positions.append(selected_pose[np.argmin(mse_all)][0])
        for j in range(1, 5):
            my_pt.positions.append(initial_position[j])
        
        my_pt.time_from_start.secs = tfs
        tfs += 30
        joint_traj.points.append(my_pt)
        num_msg += 1
        
        my_pt = JointTrajectoryPoint()
        for j in range(0, 5):
            my_pt.positions.append(selected_pose[np.argmin(mse_all)][j])
        
        my_pt.time_from_start.secs = tfs
        tfs += 30
        joint_traj.points.append(my_pt)
        num_msg += 1

        my_pt = JointTrajectoryPoint()
        my_pt.positions.append(selected_pose[np.argmin(mse_all)][0])
        for j in range(1, 5):
            my_pt.positions.append(initial_position[j])
        
        my_pt.time_from_start.secs = tfs
        tfs += 30
        joint_traj.points.append(my_pt)
        num_msg += 1

    # For every pair of checkpoint_frames, interpolate the path with potential field:
    # for i in range(1, len(checkpoint_frames)):
    #     # Potential field:
    #     PotentialField_PathPlanning(checkpoint_frames[i - 1].p, checkpoint_frames[i].p)

    print("joint_traj: {}".format(joint_traj))
    rospy.sleep(5)
    return joint_traj
    
def main_traj(cw2data):
    rospy.init_node('youbot_traj_cw2', anonymous=True)

    # Imported class from Q4:
    my_youbot   = youbot_kinematic()

    rospack     = rospkg.RosPack()
    path        = rospack.get_path('cw2q6')

    bag         = rosbag.Bag(path + '/bags/data' + str(cw2data) + '.bag')

    # dt          = 0.01
    joint_traj  = JointTrajectory()
    
    ##TODO: Change this as appropriate
    if (int(cw2data) == 1):
        print("run q6a")
        traj_pub   = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
        joint_traj = shortest_traj(my_youbot, bag, joint_traj)
        traj_pub.publish(joint_traj)

    else:
        print("run q6b")
        traj_pub   = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
        joint_traj = obstable_traj(my_youbot, bag, joint_traj)
        traj_pub.publish(joint_traj)
        
    

if __name__ == '__main__':
    try:
        main_traj(sys.argv[1])
    except rospy.ROSInterruptException:
        pass