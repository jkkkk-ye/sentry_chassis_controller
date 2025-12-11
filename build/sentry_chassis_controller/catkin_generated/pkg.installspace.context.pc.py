# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;controller_interface;control_toolbox;hardware_interface;pluginlib;dynamic_reconfigure;tf;nav_msgs;geometry_msgs;urdf;angles;realtime_tools".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsentry_chassis_controller".split(';') if "-lsentry_chassis_controller" != "" else []
PROJECT_NAME = "sentry_chassis_controller"
PROJECT_SPACE_DIR = "/home/jkkkk/ros_ws/install"
PROJECT_VERSION = "0.1.0"
