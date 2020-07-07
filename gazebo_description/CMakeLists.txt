cmake_minimum_required(VERSION 3.0.0)
project(gazebo_description)
SET(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros  
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros 
  DEPENDS 
    GAZEBO
)

