cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(FCL REQUIRED fcl)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})

ADD_CUSTOM_TARGET( ros rosmake ${PROJECT_NAME}
	COMMAND rosrun ${PROJECT_NAME} TriangleMesh 
	COMMENT   "rosmake & rosrun"
)

rosbuild_add_library(TriangleObjectLib src/ros_util.cpp src/ros_util.h src/util.h)
rosbuild_add_executable(TriangleMesh src/triangle_visualization.cpp)
target_link_libraries(TriangleMesh TriangleObjectLib ${FCL_LIBRARIES})

rosbuild_add_executable(FootstepNode src/node_wrapper.cpp)
rosbuild_add_executable(DebugListener src/test_listener.cpp)
rosbuild_add_executable(BasicShape src/basic_shape.cpp)

