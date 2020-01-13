# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "human_moveit_config: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(human_moveit_config_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_custom_target(_human_moveit_config_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "human_moveit_config" "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" "sensor_msgs/JointState:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(human_moveit_config
  "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_moveit_config
)

### Generating Module File
_generate_module_cpp(human_moveit_config
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_moveit_config
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(human_moveit_config_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(human_moveit_config_generate_messages human_moveit_config_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_dependencies(human_moveit_config_generate_messages_cpp _human_moveit_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_moveit_config_gencpp)
add_dependencies(human_moveit_config_gencpp human_moveit_config_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_moveit_config_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(human_moveit_config
  "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_moveit_config
)

### Generating Module File
_generate_module_eus(human_moveit_config
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_moveit_config
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(human_moveit_config_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(human_moveit_config_generate_messages human_moveit_config_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_dependencies(human_moveit_config_generate_messages_eus _human_moveit_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_moveit_config_geneus)
add_dependencies(human_moveit_config_geneus human_moveit_config_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_moveit_config_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(human_moveit_config
  "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_moveit_config
)

### Generating Module File
_generate_module_lisp(human_moveit_config
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_moveit_config
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(human_moveit_config_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(human_moveit_config_generate_messages human_moveit_config_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_dependencies(human_moveit_config_generate_messages_lisp _human_moveit_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_moveit_config_genlisp)
add_dependencies(human_moveit_config_genlisp human_moveit_config_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_moveit_config_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(human_moveit_config
  "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_moveit_config
)

### Generating Module File
_generate_module_nodejs(human_moveit_config
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_moveit_config
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(human_moveit_config_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(human_moveit_config_generate_messages human_moveit_config_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_dependencies(human_moveit_config_generate_messages_nodejs _human_moveit_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_moveit_config_gennodejs)
add_dependencies(human_moveit_config_gennodejs human_moveit_config_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_moveit_config_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(human_moveit_config
  "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config
)

### Generating Module File
_generate_module_py(human_moveit_config
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(human_moveit_config_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(human_moveit_config_generate_messages human_moveit_config_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/robot_ws_zheng/src/human_moveit_config/srv/GetHumanIK.srv" NAME_WE)
add_dependencies(human_moveit_config_generate_messages_py _human_moveit_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(human_moveit_config_genpy)
add_dependencies(human_moveit_config_genpy human_moveit_config_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_moveit_config_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_moveit_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_moveit_config
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(human_moveit_config_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(human_moveit_config_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(human_moveit_config_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_moveit_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/human_moveit_config
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(human_moveit_config_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(human_moveit_config_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(human_moveit_config_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_moveit_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_moveit_config
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(human_moveit_config_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(human_moveit_config_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(human_moveit_config_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_moveit_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/human_moveit_config
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(human_moveit_config_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(human_moveit_config_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(human_moveit_config_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_moveit_config/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(human_moveit_config_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(human_moveit_config_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(human_moveit_config_generate_messages_py sensor_msgs_generate_messages_py)
endif()