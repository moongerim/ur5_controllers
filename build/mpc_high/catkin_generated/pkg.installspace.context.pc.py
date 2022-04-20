# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;roscpp;rospy;std_msgs;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmpc_high".split(';') if "-lmpc_high" != "" else []
PROJECT_NAME = "mpc_high"
PROJECT_SPACE_DIR = "/home/robot/workspaces/ur5_controllers/install"
PROJECT_VERSION = "0.0.0"
