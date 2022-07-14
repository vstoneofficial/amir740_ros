/*
 * グリッパの姿勢を固定しながら、動かすサンプル
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


const double MAX_VEL_ARM = 0.5;
const double MAX_ACC_ARM = 0.6;
const double MAX_VEL_GRIPPER = 1.0;
const double MAX_ACC_GRIPPER = 1.0;

std::vector<double> joint_group_positions;

double radians(double degrees)
{
    return degrees * M_PI / 180;
}

void moveGripper(moveit::planning_interface::MoveGroupInterface &group, double angle, double delay)
{
    std::vector<double> gripper_pose = {tf2Radians(angle),
                                        tf2Radians(angle),
                                        tf2Radians(angle),
                                        tf2Radians(angle),
                                        tf2Radians(angle),
                                        tf2Radians(angle)};
    group.setJointValueTarget(gripper_pose);
    ROS_INFO("Moving gripper");
    group.move();
    ros::Duration(delay).sleep();
}

void moveArm(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z, double roll, double pitch, double yaw, double speed)
{
    tf2::Quaternion orientation;
    orientation.setRPY(tf2Radians(roll), tf2Radians(pitch), tf2Radians(yaw));

    group.setMaxVelocityScalingFactor(speed);

    geometry_msgs::Pose pose;
    pose.orientation = tf2::toMsg(orientation);
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    group.setPoseTarget(pose);
    // group.setGoalTolerance(0.01);
    group.move();
}

void moveArmWithAngles(moveit::planning_interface::MoveGroupInterface &group, std::vector<double> &angles, int delay)
{
    std::vector<double> arm_pose = {tf2Radians(angles[0]),
                                    tf2Radians(angles[1]),
                                    tf2Radians(angles[2]),
                                    tf2Radians(angles[3]),
                                    tf2Radians(angles[4])};
    group.setJointValueTarget(arm_pose);
    ROS_INFO("Moving arm");
    group.move();
    ros::Duration(delay).sleep();
}

void moveSingleJoint(moveit::planning_interface::MoveGroupInterface &group, int jointNumber, double angle_deg, double delay)
{
    ROS_INFO("moving joint %d to %.2f deg", jointNumber + 1, angle_deg);
    joint_group_positions[jointNumber] = radians(angle_deg);
    group.setJointValueTarget(joint_group_positions);
    group.move();
    ROS_INFO("moved");
    // ROS_INFO("moved joint %d to %.2f deg", jointNumber+1, angle_deg);
    ros::Duration(delay).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit_visual_tools::MoveItVisualTools visual_tools("link0_1");

    // creating planning group
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    moveit::core::RobotStatePtr current_state;
    const moveit::core::JointModelGroup *joint_model_group =
        arm.getCurrentState()->getJointModelGroup("arm");

    arm.setMaxVelocityScalingFactor(MAX_VEL_ARM);
    arm.setMaxAccelerationScalingFactor(MAX_ACC_ARM);
    gripper.setMaxVelocityScalingFactor(MAX_VEL_GRIPPER);
    gripper.setMaxAccelerationScalingFactor(MAX_ACC_GRIPPER);

    // get current joint angles
    std::vector<double> home_pose = {169, 134, -159, -74, -157};
    std::vector<double> bow_pose = {-0, 134, -134, 90, 0};
    std::vector<double> extended_pose = {0, 0, 0, 0, 0};
    

    // moveArm(arm, 0.02, 0.0, 0.65, 0, -90, 0, 0.6);
    // moveArm(arm, 0.0, 0.10, 0.185 + 0.4365, -90, 0, 0, 0.6);
    // moveArmWithAngles(arm, bow_pose, 0.6);
    // current_state = arm.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


    // current_state->setFromIK(joint_model_group, target_pose);
    // arm.setStartStateToCurrentState();
    // arm.setPoseTarget(target_pose);
    // arm.setPlanningTime(10.0);


    while (ros::ok())
    {   
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to front");
        moveArm(arm, 0.0, 0.50, 0.185, -90, 0, 0, 0.6);
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move back");
        arm.setMaxVelocityScalingFactor(MAX_VEL_ARM);
        arm.setMaxAccelerationScalingFactor(MAX_ACC_ARM);
        tf2::Quaternion orientation;
        orientation.setRPY(tf2Radians(-90), tf2Radians(0), tf2Radians(0));
        geometry_msgs::Pose target_pose;
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = 0.0;
        target_pose.position.y = 0.5;
        target_pose.position.z = 0.185;

        std::vector<geometry_msgs::Pose> waypoints;
        target_pose.position.y = 0.10;
        waypoints.push_back(target_pose);


        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setMaxAccelerationScalingFactor(0.6);
        arm.execute(my_plan);

        current_state = arm.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to front");
        moveArm(arm, 0.0, 0.50, 0.185, -90, 0, 0, 0.6);

        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to front");
        arm.execute(my_plan);
    }
}