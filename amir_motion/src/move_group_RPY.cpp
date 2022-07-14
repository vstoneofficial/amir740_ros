/*
 * 第5軸の姿勢目標を（ロール、ピッチ、ヨー）形式＋目標位置を（x、y、z）形式で
 * AMIR 740を動かすサンプル
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_RPY");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm");

    double max_vel = 0.5;
    double max_accel = 0.6;
    move_group.setMaxVelocityScalingFactor(max_vel);
    move_group.setMaxAccelerationScalingFactor(max_accel);

    ROS_INFO("Begin Planning...");

    double roll = 0;  // red x
    double pitch = 0; // green y
    double yaw = 0;   // blue z

    yaw = -90;
    tf2::Quaternion orientation;
    orientation.setRPY(tf2Radians(roll), tf2Radians(pitch), tf2Radians(yaw));

    geometry_msgs::Pose pose1;
    pose1.orientation = tf2::toMsg(orientation);
    pose1.position.x = 0.25;
    pose1.position.y = 0.0;
    pose1.position.z = 0.25;
    move_group.setPoseTarget(pose1);
    move_group.setGoalTolerance(0.01);
    move_group.move();

    ROS_INFO("Done moving...");
}