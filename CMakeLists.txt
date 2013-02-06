cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)
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
	COMMAND rosrun ${PROJECT_NAME} Main 
	COMMENT "rosmake & rosrun"
)


set(MAIN_EXECUTABLE Main)
ADD_CUSTOM_TARGET( run
	COMMAND ./../bin/${MAIN_EXECUTABLE}
	COMMENT "execute main project"
)

rosbuild_add_library(TriangleObjectLib src/ros_util.cpp src/ros_util.h src/util.h)

##### UNIT TESTING #####################
################################################################################
rosbuild_add_executable(UnitTestDistance src/unit_testing_distance_checker.cpp)
target_link_libraries(UnitTestDistance TriangleObjectLib ${FCL_LIBRARIES})
rosbuild_add_executable(UnitTestVisualization src/unit_testing_triangle_visualization.cpp)
target_link_libraries(UnitTestVisualization TriangleObjectLib ${FCL_LIBRARIES})
################################################################################

##### MAIN PROJECTS #####################
################################################################################
rosbuild_add_executable(${MAIN_EXECUTABLE} src/main.cpp)
target_link_libraries(${MAIN_EXECUTABLE} TriangleObjectLib ${FCL_LIBRARIES})
################################################################################

##### DEPRECATED #####################
################################################################################
rosbuild_add_executable(FootstepNode src/node_wrapper.cpp)
rosbuild_add_executable(DebugListener src/test_listener.cpp)
rosbuild_add_executable(BasicShape src/basic_shape.cpp)
################################################################################
