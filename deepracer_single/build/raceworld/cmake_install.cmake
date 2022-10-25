# Install script for directory: /home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/raceworld.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld/cmake" TYPE FILE FILES
    "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/raceworldConfig.cmake"
    "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/raceworldConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld" TYPE FILE FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/cam_raw.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/openvino_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/qrtag_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/key_op.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/Key_Cam.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/Auto_Cam.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/servo_commands1.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow_smart.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow_raceworld1.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow_raceworld2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow_stupid.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/follow_switch.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/lane_sim.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/lane.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/control_servo.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/raceworld" TYPE PROGRAM FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/build/raceworld/catkin_generated/installspace/tag_detect.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld/config" TYPE DIRECTORY FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld/launch" TYPE DIRECTORY FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld/meshes" TYPE DIRECTORY FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/raceworld/urdf" TYPE DIRECTORY FILES "/home/bughht/deepracer/Electro_RaceCar/deepracer_single/src/raceworld/urdf/")
endif()

