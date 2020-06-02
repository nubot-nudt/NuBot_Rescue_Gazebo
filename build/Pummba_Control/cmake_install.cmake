# Install script for directory: /home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Control

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/nubot_pummba_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_control/cmake" TYPE FILE FILES
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/nubot_pummba_controlConfig.cmake"
    "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/nubot_pummba_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_pummba_control" TYPE FILE FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/src/Pummba_Control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nubot_pummba_control" TYPE PROGRAM FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/Pummba_StateSub_Node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nubot_pummba_control" TYPE PROGRAM FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/Pummba_CmdPub_Node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nubot_pummba_control" TYPE PROGRAM FILES "/home/nubot/Documents/Study/GazeboStudy/Tracked-Vehicle/Nubot_Pummba_Gazebo/build/Pummba_Control/catkin_generated/installspace/Pummba_Control.py")
endif()

