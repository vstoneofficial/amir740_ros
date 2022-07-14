/*
 * 簡易なpick and placeのサンプル
 */

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// const double DEG_TO_RAD = M_PI / 180.0;
const double BASE_HEIGHT = 0.248;
const std::vector<double> OBJECT_DIMENSION = {0.075, 0.03, 0.13}; // x, y, z
const std::vector<double> OBJECT_POSITION = {0.3, 0.0, 0.0 - BASE_HEIGHT};      // x, y, z

void controlGripper(moveit::planning_interface::MoveGroupInterface &group, double angle)
{
    moveit::core::RobotStatePtr gripper_current_state;
    std::vector<double> gripper_close_angle = {tf2Radians(angle),
                                               tf2Radians(angle),
                                               tf2Radians(angle),
                                               tf2Radians(angle),
                                               tf2Radians(angle),
                                               tf2Radians(angle)};
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    gripper_current_state = group.getCurrentState();
    group.setJointValueTarget(gripper_close_angle);
    group.setGoalTolerance(0.01);
    success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Visualizing gripper_plan %s", success ? "" : "FAILED");
        ros::Duration(0.5).sleep();
        if (angle == -60)
            ROS_INFO("opening gripper");
        else if (angle == -15)
            ROS_INFO("closing gripper a little");

        group.move();
    }
}

void attachObject()
{
    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::CollisionObject grasping_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ROS_INFO("Attaching object grasping_object to robot's body");
    grasping_object.id = "object";
    attached_object.link_name = "gripper_base_1";
    attached_object.object = grasping_object;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

void detachObject()
{
    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::CollisionObject grasping_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ROS_INFO("Detaching object grasping_object to robot's body");
    grasping_object.operation = grasping_object.REMOVE;
    attached_object.link_name = "gripper_base_1";
    attached_object.object = grasping_object;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

void move(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z, double roll, double pitch, double yaw, double speed)
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
    group.setGoalTolerance(0.01);
    group.move();
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // Create vector to hold 2 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "link0_1";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 2.0;
    collision_objects[0].primitives[0].dimensions[1] = 0.9;
    collision_objects[0].primitives[0].dimensions[2] = 0.6;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = -0.39;
    collision_objects[0].primitive_poses[0].position.z = -collision_objects[0].primitives[0].dimensions[2] / 2 - BASE_HEIGHT;

    collision_objects[0].operation = collision_objects[0].ADD;

    // Define the object that we will be manipulating
    collision_objects[1].id = "object";
    collision_objects[1].header.frame_id = "link0_1";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = OBJECT_DIMENSION[0];
    collision_objects[1].primitives[0].dimensions[1] = OBJECT_DIMENSION[1];
    collision_objects[1].primitives[0].dimensions[2] = OBJECT_DIMENSION[2];

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = OBJECT_POSITION[0];
    collision_objects[1].primitive_poses[0].position.y = OBJECT_POSITION[1];
    collision_objects[1].primitive_poses[0].position.z = OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amir_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(0.5).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    arm.setPlanningTime(30.0);

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(0.5).sleep();

    arm.setSupportSurfaceName("table1");
    
    ROS_INFO("move arm to starting pose");
    move(arm, 0.0, 0.25, 0.3, 0, 0, 0, 0.6);
    ROS_INFO("move gripper to position above object");
    move(arm, OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.2, -90, 0, -90, 0.6);
    controlGripper(gripper, -60.0);
    ROS_INFO("lower gripper slowly");
    move(arm, OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.15, -90, 0, -90, 0.3);
    controlGripper(gripper, -15.0);
    attachObject();
    ROS_INFO("lift up object slowly");
    move(arm, OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.3, -90, 0, -90, 0.3);
    ROS_INFO("move to opposite side (normal speed)");
    move(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.3, -90, 0, 90, 0.6);
    ROS_INFO("place object slowly");
    move(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.16, -90, 0, 90, 0.3);
    controlGripper(gripper, -60.0);
    detachObject();
    ROS_INFO("raise gripper a little");
    move(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1] - 0.004, OBJECT_POSITION[2] + OBJECT_DIMENSION[2] / 2 + 0.24, -90, 0, 90, 0.3);
    ROS_INFO("move arm back to starting position");
    move(arm, 0.0, 0.25, 0.3, 0, 0, 0, 0.6);
    ROS_INFO("done");

    ros::waitForShutdown();
    return 0;
}