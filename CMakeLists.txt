cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
#set(ROS_COMPILE_FLAGS "-O3 -Wextra")
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

#add_definitions("-DPQP_COLLISION_CHECKING")
#include_directories("/opt")
#link_directories("/opt/openrobots")
#set(MISC
#	PQP
#	fast-replanning
#)

pkg_check_modules(FPlanner fast-replanning)
include_directories(${FPlanner_INCLUDE_DIRS})
link_directories(${FPlanner_LIBRARY_DIRS})
if(${FPlanner_FOUND})
	add_definitions("-DPQP_COLLISION_CHECKING")
	message(STATUS "------------------- ")
	message(STATUS ${FPlanner_LIBRARY_DIRS})
	message(STATUS ${FPlanner_LIBRARIES})
	message(STATUS ${FPlanner_INCLUDE_DIRS})
	message(STATUS "------------------- ")
else()
	message(STATUS "Compilation without fast-planner libary")
endif()

## pkg_check_modules(<NAME> REQUIRED <pkglib>)
## be sure that <NAME> != <pkglib>

##########################################################
##########################################################
pkg_check_modules(FCLLIB fcl)
include_directories(${FCLLIB_INCLUDE_DIRS})
link_directories(${FCLLIB_LIBRARY_DIRS})
if(${FCLLIB_FOUND})
	add_definitions("-DFCL_COLLISION_CHECKING")
	message(STATUS "------------------- ")
	message(STATUS ${FCLLIB_LIBRARY_DIRS})
	message(STATUS ${FCLLIB_LIBRARIES})
	message(STATUS ${FCLLIB_INCLUDE_DIRS})
else()
	message(STATUS "Compilation without FCL libary")
endif()
##########################################################
##########################################################
pkg_check_modules(EIGEN eigen3)
include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
if(${EIGEN_FOUND})
else()
	message(STATUS "Compilation without EIGEN libary")
endif()

#ADD_CUSTOM_TARGET( ros rosmake ${PROJECT_NAME}
#COMMAND rosrun ${PROJECT_NAME} Main 
#COMMENT "rosmake & rosrun"
#)

set(MAIN_EXE FootStepPlanner)

#ADD_CUSTOM_TARGET( run
#COMMAND ./../bin/${MAIN}
#COMMENT "execute main project"
#)
set(SOURCE
	src/environment.h
	src/environment.cpp
	src/environment_sallebauzil.cpp
	src/motionplanner.h
	src/motionplannerperrin.h
	src/motionplannerhyperplanar.h
	src/rviz/rviz_visualmarker.h
	src/rviz/rviz_visualmarker.cpp
	src/rviz/rviz_geometry.cpp
	src/rviz/rviz_visualmarker_interface.cpp
	src/rviz/rviz_visualmarker_triangleobject.cpp
	src/rviz/rviz_visualmarker_footmarker.cpp
	src/rviz/rviz_visualmarker_mesh.cpp
	src/rviz/rviz_visualmarker_common.cpp
	src/util.h 
	src/util.cpp
	src/util_timer.h 
	src/util_timer.cpp
	src/sampler/sampler.h
	src/sampler/sampler.cpp
	src/sampler/sampler_abstract_problem.h
	src/sampler/sampler_cylinder.cpp
	src/sampler/sampler_proposal.cpp
	src/contact_transition.h 
	src/contact_transition.cpp 
)
if(${FPlanner_FOUND})
	set(SOURCE ${SOURCE} src/motionplannerperrin.h)
endif()
add_definitions("-std=c++0x")
include_directories(src)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lfcl -L/usr/local/bin")
set(UTIL_LIB util)
rosbuild_add_library(${UTIL_LIB} ${SOURCE})
target_link_libraries(${UTIL_LIB} ${FCLLIB_LIBRARIES})
#target_link_libraries(${UTIL_LIB} ${FCLLIB_LIBRARY_DIRS})


################################################################################
###### MAIN PROJECTS #####################
################################################################################
#rosbuild_add_executable(${MAIN_EXE} src/main.cpp )
#target_link_libraries(${MAIN_EXE} ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
################################################################################
##### PROJECTS #####################
################################################################################
#rosbuild_add_executable(MCMCSampler test/mcmc_main.cpp )
#target_link_libraries(MCMCSampler ${UTIL_LIB} ${FCLLIB_LIBRARIES})

#rosbuild_add_executable(MOCAPTester src/mocap_main.cpp )
#target_link_libraries(MOCAPTester ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})

rosbuild_add_executable(ENVTester test/env_main.cpp )
target_link_libraries(ENVTester ${UTIL_LIB} ${FCLLIB_LIBRARIES} ${FPlanner_LIBRARIES})

#rosbuild_add_executable(VisSteps src/vis_steps.cpp )
#target_link_libraries(VisSteps ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
#################################################################################
###### UNIT TESTING #####################
#################################################################################
#
#if(${FPlanner_FOUND})
#rosbuild_add_executable(UnitTestFastPlannerLibrary test/unit_testing_exec_replanning.cpp )
#target_link_libraries(UnitTestFastPlannerLibrary ${UTIL_LIB} ${FPlanner_LIBRARIES} ${FPlanner_LIBRARIES})
#endif()
#

###### DEPRECATED #####################
################################################################################
#rosbuild_add_executable(FootstepNode src/node_wrapper.cpp)
#rosbuild_add_executable(DebugListener src/test_listener.cpp)
#rosbuild_add_executable(BasicShape src/basic_shape.cpp)
################################################################################
