cmake_minimum_required(VERSION 2.8.3)
project(ros_emergency_stop_recover)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    dynamic_reconfigure
    tf
    sensor_msgs
    std_msgs
    message_generation
    tf2_ros
    nav_msgs
    tf2
    roslint
)

generate_dynamic_reconfigure_options(
    config/params.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp dynamic_reconfigure sensor_msgs
)

include_directories(
  SYSTEM 
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ros_emergency_stop_recover
    src/emergency_stop_recover.cpp
)
add_dependencies(ros_emergency_stop_recover
    ${tiago_pcl_tutorial_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ros_emergency_stop_recover_gencfg
)
target_link_libraries(ros_emergency_stop_recover
    ${catkin_LIBRARIES}
)

roslint_cpp()
