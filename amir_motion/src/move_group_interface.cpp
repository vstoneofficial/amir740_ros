/*
 * Move Group C++ Interfaceの使い方のサンプル
 * こちらを参照して作成したサンプル
 * https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>

// const double tau = 2 * M_PI;
// const double DEG_TO_RAD = M_PI / 180.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //////////////////////////
    // SETUP
    //////////////////////////
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("dummy_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.5;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
        move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    /////////////////////////////
    // Planning to a Pose goal
    /////////////////////////////
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    tf2::Quaternion orientation;
    orientation.setRPY(tf2Radians(-90.0), tf2Radians(1e-6), tf2Radians(1e-6));

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 1e-6;
    target_pose1.position.y = 0.18;
    target_pose1.position.z = 0.20;
    move_group_interface.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move");
    move_group_interface.move();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    //////////////////////////////////
    // Planning to a joint-space goal
    //////////////////////////////////

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -M_PI / 2;  // move joint1 -90 deg (pi / 2 rad)
    move_group_interface.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");


    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ///////////////////////////////////
    // Planning with Path Constraints
    ///////////////////////////////////
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "tcp_link";
    ocm.header.frame_id = "dummy_link";
    ocm.orientation = tf2::toMsg(orientation);
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    // move_group_interface.setPathConstraints(test_constraints); // TODO: check why this isnt working, maybe not enough DOF??

    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation = tf2::toMsg(orientation);
    start_pose2.position.x = 0.0;
    start_pose2.position.y = 0.45;
    start_pose2.position.z = 0.25;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group_interface.setStartState(start_state);

    move_group_interface.setPoseTarget(target_pose1);

    move_group_interface.setPlanningTime(10.0);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // move_group_interface.clearPathConstraints();

    ///////////////////////////////////
    // Cartesian Paths
    ///////////////////////////////////

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2); // comment this out to prevent error..Trajectory message contains waypoints that are not strictly increasing in time
    
    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.05;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y -= 0.25;
    waypoints.push_back(target_pose3);  // left

    // orientation.setRPY(tf2Radians(0), tf2Radians(0), tf2Radians(0));
    // target_pose3.orientation = tf2::toMsg(orientation);
    // target_pose3.position.z += 0.05;
    // target_pose3.position.y += 0.20;
    // // target_pose3.position.x += 0.20;
    // waypoints.push_back(target_pose3);  // up and left


    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    my_plan.trajectory_ = trajectory;
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
    // Pull requests are welcome.
    //
    // You can execute a trajectory like this.
    move_group_interface.execute(my_plan);

    ///////////////////////////////////////
    // without obstacles
    ///////////////////////////////////////
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    geometry_msgs::Pose another_pose;
    orientation.setRPY(tf2Radians(-90), tf2Radians(0), tf2Radians(0));
    another_pose.orientation = tf2::toMsg(orientation);
    another_pose.position.x = 0.0;
    another_pose.position.y = 0.50;
    another_pose.position.z = 0.20;
    move_group_interface.setPoseTarget(another_pose);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    ///////////////////////////////////////
    // Adding objects to the environment
    ///////////////////////////////////////
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.5;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.25;
    box_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // Now when we plan a trajectory it will avoid the obstacle
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    ///////////////////////////////////////
    // Attaching objects to the robot
    ///////////////////////////////////////
    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.01;

    // We define the frame/pose for this cylinder so that it appears in the gripper
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    orientation.setRPY(tf2Radians(90), tf2Radians(0), tf2Radians(0));
    grab_pose.orientation = tf2::toMsg(orientation);
    grab_pose.position.z = 0.0;
    grab_pose.position.y = 0.10;


    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group_interface.attachObject(object_to_attach.id, "gripper_base_1");

    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    // Replan, but now with the object in hand.
    move_group_interface.setStartStateToCurrentState();
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    ///////////////////////////////////////
    // Detaching and Removing Objects
    ///////////////////////////////////////
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group_interface.detachObject(object_to_attach.id);

    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

    // Now, let's remove the objects from the world.
    ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

    ros::shutdown();
    return 0;

}
