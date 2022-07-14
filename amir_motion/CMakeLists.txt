cmake_minimum_required(VERSION 3.0.2)
project(amir_motion)

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amir_motion
#  CATKIN_DEPENDS moveit_msgs moveit_ros_planning_interface moveit_visual_tools roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning_scene_primatives
  src/planning_scene_primatives.cpp
)
add_dependencies(planning_scene_primatives ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_primatives
  ${catkin_LIBRARIES}
)

add_executable(move_group_interface
  src/move_group_interface.cpp
)
add_dependencies(move_group_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_interface
  ${catkin_LIBRARIES}
)

add_executable(pick_place
  src/pick_place.cpp
)
add_dependencies(pick_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place
  ${catkin_LIBRARIES}
)

add_executable(move_group_RPY
  src/move_group_RPY.cpp
)
add_dependencies(move_group_RPY ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_RPY
  ${catkin_LIBRARIES}
)

add_executable(constrain
  src/constrain.cpp
)
add_dependencies(constrain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(constrain
  ${catkin_LIBRARIES}
)

add_executable(move_group_joint_value
  src/move_group_joint_value.cpp
)
add_dependencies(move_group_joint_value ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_joint_value
  ${catkin_LIBRARIES}
)