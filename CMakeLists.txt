cmake_minimum_required(VERSION 3.0.2)
project(emergency_stop_recover_pa)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure

    pcl_ros
    pcl_conversions

    std_msgs
    sensor_msgs

    roslint
)

generate_dynamic_reconfigure_options(
    config/params.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    dynamic_reconfigure

    std_msgs
    sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(ros_emergency_stop_recover
    src/emergency_stop_recover.cpp
)
add_dependencies(ros_emergency_stop_recover
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(ros_emergency_stop_recover
    ${catkin_LIBRARIES}
)

roslint_cpp()
