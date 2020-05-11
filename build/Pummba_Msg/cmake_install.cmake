# Install script for directory: /home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Msg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/msg" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Msg/msg/FlipCmd.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/cmake" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msg-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/include/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/share/roseus/ros/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/share/common-lisp/ros/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/share/gennodejs/ros/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/lib/python2.7/dist-packages/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/devel/lib/python2.7/dist-packages/nubot_pummba_msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/cmake" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msg-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/cmake" TYPE FILE FILES
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msgConfig.cmake"
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Msg/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/cmake" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msg-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg/cmake" TYPE FILE FILES
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msgConfig.cmake"
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Msg/catkin_generated/installspace/nubot_pummba_msgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_msg" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Msg/package.xml")
endif()

