set(iDynTree_VERSION 0.11.1)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was iDynTreeConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

set_and_check(iDynTree_INCLUDEDIR "")

if(NOT TARGET iDynTree::idyntree-core)
  include("${CMAKE_CURRENT_LIST_DIR}/iDynTreeTargets.cmake")
endif()

# Compatibility
set(iDynTree_LIBRARIES iDynTree::idyntree-core;iDynTree::idyntree-model;iDynTree::idyntree-sensors;iDynTree::idyntree-modelio-xml;iDynTree::idyntree-modelio-urdf;iDynTree::idyntree-estimation;iDynTree::idyntree-high-level;iDynTree::idyntree-inverse-kinematics;iDynTree::idyntree-high-level-kdl;iDynTree::idyntree-regressors;iDynTree::idyntree-visualization)
set(iDynTree_INCLUDE_DIRS ${iDynTree_INCLUDEDIR})
