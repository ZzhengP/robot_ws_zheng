# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "cv_bridge;geometry_msgs;image_transport;kdl_conversions;roscpp;roslib;tf;tf_conversions;visualization_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lkinect2_tracker".split(';') if "-lkinect2_tracker" != "" else []
PROJECT_NAME = "kinect2_tracker"
PROJECT_SPACE_DIR = "/home/zheng/robot_ws_zheng/install"
PROJECT_VERSION = "0.0.1"
