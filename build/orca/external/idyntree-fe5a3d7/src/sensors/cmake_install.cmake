# Install script for directory: /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zheng/robot_ws_zheng/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/zheng/robot_ws_zheng/build/lib/libidyntree-sensors.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Sensors" TYPE FILE FILES
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/AllSensorsTypes.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/Sensors.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/SixAxisForceTorqueSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/SixAxisFTSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/GyroscopeSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/AccelerometerSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/PredictSensorsMeasurements.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include/iDynTree/Sensors/ModelSensorsTransformers.h"
    )
endif()

