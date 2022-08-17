/*
 * 各軸の目標角度を指定してAMIR 740を動かすサンプル
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// 最大関節速度、加速度を下げるためのスケーリング係数の設定。許可される値は0～1です。
// ロボットモデルで指定された最大関節速度に係数が乗算されます。
const double MAX_VEL_ARM = 0.8;         // アームの最大速度の割合
const double MAX_ACC_ARM = 0.5;         // アームの最大速度の割合
const double MAX_VEL_GRIPPER = 1.0;     // アームの最大速度の割合
const double MAX_ACC_GRIPPER = 1.0;     // アームの最大速度の割合

void moveArm(moveit::planning_interface::MoveGroupInterface& group, std::vector<double>& angles, double delay, double maxVelScalingFactor)
{
    std::vector<double> arm_pose = {tf2Radians(angles[0]), 
                                    tf2Radians(angles[1]),
                                    tf2Radians(angles[2]),
                                    tf2Radians(angles[3]),
                                    tf2Radians(angles[4])};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(arm_pose);
    ROS_INFO("Moving arm");
    group.move();
    ros::Duration(delay).sleep();
}

void moveGripper(moveit::planning_interface::MoveGroupInterface& group, double angle, double delay, double maxVelScalingFactor)
{
    std::vector<double> gripper_pose = {tf2Radians(angle), 
                                    tf2Radians(angle),
                                    tf2Radians(angle),
                                    tf2Radians(angle),
                                    tf2Radians(angle),
                                    tf2Radians(angle)};
    group.setMaxVelocityScalingFactor(maxVelScalingFactor);
    group.setJointValueTarget(gripper_pose);
    ROS_INFO("Moving gripper");
    group.move();
    ros::Duration(delay).sleep();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_group_joint_value_sample");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit_visual_tools::MoveItVisualTools visual_tools("link0_1");

    // creating planning group
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    arm.setMaxVelocityScalingFactor(MAX_VEL_ARM);
    arm.setMaxAccelerationScalingFactor(MAX_ACC_ARM);

    // get current joint angles
    std::vector<double> home_pose = {170, 135, -160, 75, -158};
    std::vector<double> ready_pose = {-0, 135, -135, 0, 0};
    std::vector<double> joint1_max = {170, 135, -135, 0, 0};
    std::vector<double> joint1_min = {-170, 135, -135, 0, 0};
    std::vector<double> joint3_folded = {-0, 135, -160, 0, 0};
    std::vector<double> joint3_extended = {-0, 135, -0, 0, 0};
    std::vector<double> extended_pose = {0, 0, 0, 0, 0};
    std::vector<double> joint4_down = {0, 0, 0, -120, 0};
    std::vector<double> joint4_up = {0, 0, 0, 75, 0};
    std::vector<double> joint5_min = {0, 0, 0, 0, -158};
    std::vector<double> joint5_max = {0, 0, 0, 0, 158};

    while (ros::ok())
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        moveArm(arm, ready_pose, 0.5, 0.5);
        ROS_INFO("testing joint1 range");
        moveArm(arm, joint1_max, 0.0, 0.6);
        moveArm(arm, joint1_min, 0.0, 0.6);
        ROS_INFO("testing joint3 range");
        moveArm(arm, joint3_folded, 0.3, 0.5);
        moveArm(arm, joint3_extended, 0.0, 0.5);
        ROS_INFO("testing joint2 range");
        moveArm(arm, extended_pose, 0.5, 0.5);
        ROS_INFO("testing joint4 range");
        moveArm(arm, joint4_down, 0.0, 0.7);
        moveArm(arm, joint4_up, 0.0, 0.7);
        moveArm(arm, extended_pose, 0.5, 0.7);
        ROS_INFO("testing joint5 range");
        moveArm(arm, joint5_min, 0.0, 0.7);
        moveArm(arm, joint5_max, 0.0, 0.7);
        moveArm(arm, extended_pose, 0.5, 0.7);
        ROS_INFO("close gripper");
        moveGripper(gripper, 14, 0.5, MAX_VEL_GRIPPER);
        ROS_INFO("open gripper");
        moveGripper(gripper, -59, 0.5, MAX_VEL_GRIPPER);
        ROS_INFO("done");

    }
}