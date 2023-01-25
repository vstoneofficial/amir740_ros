cmake_minimum_required(VERSION 3.0.2)
project(amir_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
