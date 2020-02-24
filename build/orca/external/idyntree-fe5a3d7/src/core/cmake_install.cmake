# Install script for directory: /home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/zheng/robot_ws_zheng/build/lib/libidyntree-core.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iDynTree/Core" TYPE FILE FILES
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Axis.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/ArticulatedBodyInertia.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/ClassicalAcc.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Direction.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/EigenSparseHelpers.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/EigenMathHelpers.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/EigenHelpers.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/InertiaNonLinearParametrization.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/MatrixDynSize.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/MatrixFixSize.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Position.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PositionRaw.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PositionSemantics.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Rotation.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/RotationRaw.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/RotationSemantics.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/RotationalInertiaRaw.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialAcc.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialForceVector.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialInertiaRaw.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialInertia.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialMomentum.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialMotionVector.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/TestUtils.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Transform.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/TransformDerivative.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/TransformSemantics.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Twist.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Utils.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/VectorFixSize.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/VectorDynSize.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/LinearForceVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/AngularForceVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/LinearMotionVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/AngularMotionVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Wrench.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PrivateUtils.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PrivateMotionForceVertorAssociations.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PrivatePreProcessorUtils.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/PrivateSemanticsMacros.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/GeomVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/MotionVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/ForceVector3.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SpatialVector.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/SparseMatrix.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Triplets.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/CubicSpline.h"
    "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include/iDynTree/Core/Span.h"
    )
endif()

