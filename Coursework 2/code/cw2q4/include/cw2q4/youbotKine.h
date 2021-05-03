#ifndef CW2Q4_CW2Q4_H
#define CW2Q4_CW2Q4_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

using namespace Eigen;

class youbot_kinematic
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    tf2_ros::TransformBroadcaster pose_br;
    double DH_params[5][4];
    double joint_limit_min[5];
    double joint_limit_max[5];
    double joint_offset[5];
    Matrix4d current_pose;
    double current_joint_position[5];

public:
    void init();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    Matrix4d forward_kine(double joint_val[], int frame);
    Matrix4d forward_kine_offset(double joint_val[], int frame);
    void broadcast_pose(Matrix4d pose);
    MatrixXd get_jacobian(double joint_val[]);
    MatrixXd inverse_kine_closed_form(Matrix4d pose);
    double* inverse_kine_ite(Matrix4d pose, double joint_val[]);
    bool check_singularity(double joint_val[]);
    Matrix4d dh_matrix_standard(double a, double alpha, double d, double theta);
};


#endif //CW2Q4_CW2Q4_H
