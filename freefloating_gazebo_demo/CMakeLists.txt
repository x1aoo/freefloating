cmake_minimum_required(VERSION 2.8.3)
project(freefloating_gazebo_demo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
)

install(DIRECTORY
  launch
  objects
  scenes
  scripts
  terrain
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})

# demo node
add_executable(freefloating_gazebo_demo src/freefloating_gazebo_demo.cpp)
target_link_libraries(freefloating_gazebo_demo ${catkin_LIBRARIES})
#vectored node
 add_executable(velocity_torque  src/velocity_torque.cpp)
 target_link_libraries(velocity_torque ${catkin_LIBRARIES})
