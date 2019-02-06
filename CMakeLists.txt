cmake_minimum_required(VERSION 2.8.3)
project(ifco_pose_estimator)

message(STATUS "using ros $ENV{ROS_DISTRO}")

if(($ENV{ROS_DISTRO} STREQUAL "melodic"))
    message(STATUS "adding -std=c++11")
    add_compile_options(-std=c++11)
endif()
if(($ENV{ROS_DISTRO} STREQUAL "lunar"))
    message(STATUS "adding -std=c++11")
    add_compile_options(-std=c++11)
endif()
if(($ENV{ROS_DISTRO} STREQUAL "kinetic"))
    message(STATUS "adding -std=c++11")
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  roscpp
  std_msgs
  cv_bridge
  pcl_ros
  tf
  moveit_core
  moveit_ros_planning_interface
  shape_msgs
  moveit_msgs
  tf_conversions
)

add_service_files(FILES
  ifco_pose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}

)

add_executable(ifco_pose_server
  src/ifco_pose_server.cpp
)
add_dependencies(ifco_pose_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ifco_pose_server
  ${catkin_LIBRARIES}
)

add_executable(ifco_pose_client
  src/ifco_pose_client.cpp
)
add_dependencies(ifco_pose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ifco_pose_client
  ${catkin_LIBRARIES}
)
