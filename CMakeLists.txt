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

include(CheckFunctionExists)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(FPlanner REQUIRED fast-replanning)
include_directories(${FPlanner_INCLUDE_DIRS})
link_directories(${FPlanner_LIBRARY_DIRS})

## pkg_check_modules(<NAME> REQUIRED <pkglib>)
## be sure that <NAME> != <pkglib>
pkg_check_modules(FCLLIB REQUIRED fcl)
include_directories(${FCLLIB_INCLUDE_DIRS})
link_directories(${FCLLIB_LIBRARY_DIRS})

#ADD_CUSTOM_TARGET( ros rosmake ${PROJECT_NAME}
#COMMAND rosrun ${PROJECT_NAME} Main 
#COMMENT "rosmake & rosrun"
#)

set(MAIN_EXE MainProject)
message(STATUS "DEBUG OUTPUT ---- ")
message(STATUS "")
message(STATUS ${FPlanner_LIBRARY_DIRS})
message(STATUS ${FPlanner_LIBRARIES})
message(STATUS ${FPlanner_INCLUDE_DIRS})
message(STATUS "------------------- ")
message(STATUS ${FCLLIB_LIBRARY_DIRS})
message(STATUS ${FCLLIB_LIBRARIES})
message(STATUS ${FCLLIB_INCLUDE_DIRS})
message(STATUS "------------------- ")

#ADD_CUSTOM_TARGET( run
#COMMAND ./../bin/${MAIN}
#COMMENT "execute main project"
#)
set(SOURCE
	src/ros_util.cpp 
	src/ros_util.h 
	src/util.h 
)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lfcl -L/usr/local/bin")
set(UTIL_LIB util)
rosbuild_add_library(${UTIL_LIB} ${SOURCE})
target_link_libraries(${UTIL_LIB} ${FCLLIB_LIBRARIES})
#target_link_libraries(${UTIL_LIB} ${FCLLIB_LIBRARY_DIRS})



################################################################################
###### MAIN PROJECTS #####################
################################################################################
rosbuild_add_executable(${MAIN_EXE} src/main.cpp )
target_link_libraries(${MAIN_EXE} ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
################################################################################
##### UNIT TESTING #####################
################################################################################
rosbuild_add_executable(UnitTestDistance src/unit_testing_distance_checker.cpp)
target_link_libraries(UnitTestDistance ${UTIL_LIB} ${FCLLIB_LIBRARIES})

rosbuild_add_executable(UnitTestVisualization src/unit_testing_triangle_visualization.cpp )
target_link_libraries(UnitTestVisualization ${UTIL_LIB} ${FCLLIB_LIBRARIES})

rosbuild_add_executable(UnitTestFastPlannerLibrary src/unit_testing_exec_replanning.cpp )
target_link_libraries(UnitTestFastPlannerLibrary ${UTIL_LIB} ${FPlanner_LIBRARIES} ${FPlanner_LIBRARIES})

rosbuild_add_executable(UnitTestMCMCSampling src/unit_testing_mcmc_sampling.cpp )
target_link_libraries(UnitTestMCMCSampling ${UTIL_LIB} ${FPlanner_LIBRARIES})

###### DEPRECATED #####################
################################################################################
#rosbuild_add_executable(FootstepNode src/node_wrapper.cpp)
#rosbuild_add_executable(DebugListener src/test_listener.cpp)
#rosbuild_add_executable(BasicShape src/basic_shape.cpp)
################################################################################
