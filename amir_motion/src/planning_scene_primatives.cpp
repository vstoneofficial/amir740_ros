/*
 * プラニングシーンのサンプル
 * AMIR 740を机上に表示するノード
 */

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
const double BASE_HEIGHT = 0.248;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 1 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "link0_1";

  // Define the primitive and its dimensions. //
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.2; 
  collision_objects[0].primitives[0].dimensions[1] = 0.6;
  collision_objects[0].primitives[0].dimensions[2] = 0.6;

  // Define the pose of the table. //
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = collision_objects[0].primitives[0].dimensions[1]/2 - 0.06;
  collision_objects[0].primitive_poses[0].position.z = -collision_objects[0].primitives[0].dimensions[2] / 2 - BASE_HEIGHT;

  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_scene_primatives");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");

  addCollisionObjects(planning_scene_interface);

  ros::waitForShutdown();
  return 0;
}