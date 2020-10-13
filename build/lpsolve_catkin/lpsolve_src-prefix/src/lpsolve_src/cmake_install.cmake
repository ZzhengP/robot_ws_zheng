# Install script for directory: /home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zheng/robot_ws_zheng/devel/.private/lpsolve_catkin")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/liblpsolve.so")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lp_solve" TYPE FILE FILES
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/hbio.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/lusol.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/lusolio.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/lusolmain.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/mmio.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/myblas.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/LUSOL/sparselib.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/bfp_LUSOL.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/bfp_LUSOL/lp_LUSOL.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/bfp/lp_BFP.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/colamd/colamd.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/declare.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/fortify.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/ini.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_Hash.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_MDO.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_MPS.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_SOS.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_bit.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_crash.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_explicit.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_fortify.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_lib.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_matrix.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_mipbb.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_presolve.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_price.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_pricePSE.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_report.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_rlp.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_scale.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_simplex.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_solveDLL.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_types.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_utils.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lp_wlp.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lpkit.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lpsolve.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/lpsolve55/resource.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/shared/commonlib.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/shared/mmio.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/shared/myblas.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/ufortify.h"
    "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/yacc_read.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zheng/robot_ws_zheng/build/lpsolve_catkin/lpsolve_src-prefix/src/lpsolve_src/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
