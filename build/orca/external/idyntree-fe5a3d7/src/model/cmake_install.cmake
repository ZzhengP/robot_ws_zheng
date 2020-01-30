# Install script for directory: /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlibx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/zheng/robot_ws_zheng/build/lib/libidyntree-model.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Model" TYPE FILE FILES
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/ContactWrench.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/DenavitHartenberg.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/FixedJoint.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/ForwardKinematics.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/FreeFloatingState.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/FreeFloatingMatrices.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/IJoint.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Dynamics.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/DynamicsLinearization.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/DynamicsLinearizationHelpers.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Indeces.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Indices.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Jacobians.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/JointState.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/LinkTraversalsCache.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Link.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/LinkState.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Model.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/ModelTransformers.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/MovableJointImpl.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/RevoluteJoint.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/PrismaticJoint.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/SolidShapes.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/SubModel.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include/iDynTree/Model/Traversal.h"
    )
endif()

