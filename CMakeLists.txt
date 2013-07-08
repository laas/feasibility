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
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/usr/local/include")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/local/lib")

FIND_PACKAGE(Boost)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

include(CheckFunctionExists)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

#add_definitions("-DPQP_COLLISION_CHECKING")
include_directories("/opt/ros/electric/stacks/visualization/interactive_markers")
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
pkg_check_modules(LibFann fann)
include_directories(${LibFann_INCLUDE_DIRS})
link_directories(${LibFann_LIBRARY_DIRS})
if(${LibFann_FOUND})
	message(STATUS "------------------- ")
	message(STATUS ${LibFann_LIBRARY_DIRS})
	message(STATUS ${LibFann_LIBRARIES})
	message(STATUS ${LibFann_INCLUDE_DIRS})
	message(STATUS "------------------- ")
else()
	message(STATUS "Compilation without fann libary")
endif()
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
	add_definitions("-DEIGEN_FOUND")
	message(STATUS "------------------- ")
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
	src/environment/environment.h
	src/environment/environment.cpp
	src/environment/environment_sallebauzil.cpp
	src/environment/environment_13humanoids.cpp
	src/environment/environment_13humanoids_real.cpp
	src/environment/environment_testbed.cpp
	src/planner/motionplanner.h
	src/planner/motionplanner_astar.h
	src/planner/constraints_checker.h 
	src/planner/constraints_checker.cpp
	src/planner/constraints_checker_ann.h 
	src/planner/constraints_checker_ann.cpp
	src/planner/constraints_checker_hyperplanar.h 
	src/planner/constraints_checker_swept_volumer.h
	src/planner/constraints_checker_swept_volumer.cpp
	src/planner/constraints_checker_naive.h
	src/planner/constraints_checker_naive.cpp
	src/planner/constraints_checker_wallconstraints_decorator.h 
	src/planner/constraints_checker_wallconstraints_decorator.cpp
	src/planner/contact_transition.h 
	src/planner/contact_transition.cpp 
	src/rviz/visualmarker.h
	src/rviz/visualmarker.cpp
	src/rviz/geometry.cpp
	src/rviz/rviz_interface.h
	src/rviz/rviz_interface.cpp
	src/rviz/visualmarker_triangleobject.cpp
	src/rviz/visualmarker_tristriangleobject.cpp
	src/rviz/visualmarker_footmarker.cpp
	src/rviz/visualmarker_collada.cpp
	src/rviz/visualmarker_mesh.cpp
	src/rviz/visualmarker_decorator.cpp
	src/rviz/visualmarker_common.cpp
	src/rviz/cylindermarkertriangles.cpp
	src/util/util.h 
	src/util/util.cpp
	src/util/util_eigen.h 
	src/util/util_eigen.cpp
	src/util/util_timer.h 
	src/util/util_timer.cpp
	src/sampler/sampler.h
	src/sampler/sampler_algorithms.cpp
	src/sampler/sampler_algorithms_uniform.cpp
	src/sampler/sampler_problem_abstract.h
	src/sampler/sampler_problem_abstract_proposal.cpp
	src/sampler/sampler_problem_cylinder.cpp
)
if(${FPlanner_FOUND})
	set(SOURCE ${SOURCE} src/planner/motionplannerperrin.h)
endif()
set(LibFann_LIBRARIES
	fann
	m
)


add_definitions("-std=c++0x")
include_directories(src)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -lfann")
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

rosbuild_add_executable(FastRePlanner test/perrin_fastplanner.cpp )
target_link_libraries(FastRePlanner ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
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
