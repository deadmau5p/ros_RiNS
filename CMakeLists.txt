cmake_minimum_required(VERSION 3.0.2)
project(task1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    visualization_msgs
    tf2_ros
    nav_msgs
    std_msgs
    geometry_msgs
)

add_message_files(DIRECTORY msg
  FILES
  ObjectDetection.msg
  PointArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()