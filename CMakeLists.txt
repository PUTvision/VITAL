cmake_minimum_required(VERSION 3.10.2)
project(visual_landing_provider)

find_package(ament_cmake QUIET)

## ROS2
if ( ament_cmake_FOUND )

  find_package(rclpy REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(mavros_msgs REQUIRED)

  set(dependencies
    rclpy
    sensor_msgs
    std_msgs
    geometry_msgs
    mavros_msgs
  )

  include_directories(
    include
    ${ament_INCLUDE_DIRS}
  )

## ROS1
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )

  find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    mavros_msgs
    )

  catkin_package()
  catkin_python_setup()

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

endif()
