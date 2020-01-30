# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget iDynTree::idyntree-core iDynTree::idyntree-model iDynTree::idyntree-sensors iDynTree::idyntree-modelio-xml iDynTree::idyntree-modelio-urdf iDynTree::idyntree-estimation iDynTree::idyntree-high-level iDynTree::idyntree-inverse-kinematics iDynTree::idyntree-high-level-kdl iDynTree::idyntree-regressors iDynTree::idyntree-visualization)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target iDynTree::idyntree-core
add_library(iDynTree::idyntree-core STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-core PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_attribute_deprecated"
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/core/include"
)

# Create imported target iDynTree::idyntree-model
add_library(iDynTree::idyntree-model STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-model PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core"
)

# Create imported target iDynTree::idyntree-sensors
add_library(iDynTree::idyntree-sensors STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-sensors PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/sensors/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-model"
)

# Create imported target iDynTree::idyntree-modelio-xml
add_library(iDynTree::idyntree-modelio-xml STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-modelio-xml PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/xml/include;/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/xml/include/iDynTree;/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/xml/include/private"
  INTERFACE_LINK_LIBRARIES "/usr/lib/x86_64-linux-gnu/libxml2.so;\$<LINK_ONLY:iDynTree::idyntree-core>"
)

# Create imported target iDynTree::idyntree-modelio-urdf
add_library(iDynTree::idyntree-modelio-urdf STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-modelio-urdf PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/urdf/include;/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/urdf/include/deprecated;/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/model_io/urdf/include/private"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-model;iDynTree::idyntree-sensors;iDynTree::idyntree-modelio-xml"
)

# Create imported target iDynTree::idyntree-estimation
add_library(iDynTree::idyntree-estimation STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-estimation PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/estimation/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-model;iDynTree::idyntree-sensors;iDynTree::idyntree-modelio-urdf"
)

# Create imported target iDynTree::idyntree-high-level
add_library(iDynTree::idyntree-high-level STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-high-level PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/high-level/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-model;iDynTree::idyntree-sensors;iDynTree::idyntree-modelio-urdf"
)

# Create imported target iDynTree::idyntree-inverse-kinematics
add_library(iDynTree::idyntree-inverse-kinematics STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-inverse-kinematics PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/inverse-kinematics/include;/usr/include/coin"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-high-level;/usr/lib/libipopt.so;/usr/lib/x86_64-linux-gnu/libdmumps_seq.so;/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/liblapack.so;/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/libdmumps_seq.so;/usr/lib/x86_64-linux-gnu/libdl.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/gcc/x86_64-linux-gnu/7/libquadmath.so;/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/x86_64-linux-gnu/libdl.so"
)

# Create imported target iDynTree::idyntree-high-level-kdl
add_library(iDynTree::idyntree-high-level-kdl STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-high-level-kdl PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/legacy/high-level-kdl/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core"
)

# Create imported target iDynTree::idyntree-regressors
add_library(iDynTree::idyntree-regressors STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-regressors PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/regressors/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-sensors"
)

# Create imported target iDynTree::idyntree-visualization
add_library(iDynTree::idyntree-visualization STATIC IMPORTED)

set_target_properties(iDynTree::idyntree-visualization PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zheng/robot_ws_zheng/src/orca/external/idyntree-fe5a3d7/src/visualization/include"
  INTERFACE_LINK_LIBRARIES "iDynTree::idyntree-core;iDynTree::idyntree-model"
)

# Import target "iDynTree::idyntree-core" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-core APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-core.a"
  )

# Import target "iDynTree::idyntree-model" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-model APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-model PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-model.a"
  )

# Import target "iDynTree::idyntree-sensors" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-sensors APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-sensors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-sensors.a"
  )

# Import target "iDynTree::idyntree-modelio-xml" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-modelio-xml APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-modelio-xml PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-modelio-xml.a"
  )

# Import target "iDynTree::idyntree-modelio-urdf" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-modelio-urdf APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-modelio-urdf PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-modelio-urdf.a"
  )

# Import target "iDynTree::idyntree-estimation" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-estimation APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-estimation PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-estimation.a"
  )

# Import target "iDynTree::idyntree-high-level" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-high-level APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-high-level PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-high-level.a"
  )

# Import target "iDynTree::idyntree-inverse-kinematics" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-inverse-kinematics APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-inverse-kinematics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-inverse-kinematics.a"
  )

# Import target "iDynTree::idyntree-high-level-kdl" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-high-level-kdl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-high-level-kdl PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-high-level-kdl.a"
  )

# Import target "iDynTree::idyntree-regressors" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-regressors APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-regressors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-regressors.a"
  )

# Import target "iDynTree::idyntree-visualization" for configuration "Debug"
set_property(TARGET iDynTree::idyntree-visualization APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(iDynTree::idyntree-visualization PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "/home/zheng/robot_ws_zheng/build/lib/libidyntree-visualization.a"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)