cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(nubot_pumbaa_plugin)

# Find ROS
# find_package(roscpp REQUIRED)
# find_package(std_msgs REQUIRED)
# # find_package(nubot_pumbaa_msg REQUIRED)
# find_package(nubot_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  message_generation
  sensor_msgs
  geometry_msgs
  nubot_msgs
  tf
  gazebo
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
  message_runtime
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
# find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# needed for SimpleTrackedVehiclePlugin
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/deps/opende/include
  ${CMAKE_SOURCE_DIR}/deps/threadpool
  ${CMAKE_SOURCE_DIR}/deps/opende/src
)

# Tinyxml are used since the plugins are linking against gazebo_common
# which uses tinxyml. Not defining the link directory here will end up
# failures in clang.
link_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CURL_LIBDIR}
  ${CCD_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
)

# Build plugin
add_library(nubot_pumbaa_plugin SHARED NuBot_Pumbaa_Gazebo_Plugin.cc)
add_dependencies(nubot_pumbaa_plugin ${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(nubot_pumbaa_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})


