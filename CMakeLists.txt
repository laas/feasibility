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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/usr/local/include")
#SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/local/lib")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
#INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/ros.cmake)
INCLUDE(CMakeModules/FindPQP.cmake)

ADD_REQUIRED_DEPENDENCY("analyticalPG >= 0.9")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")
ADD_REQUIRED_DEPENDENCY("jrl-dynamics >= 1.19.1")
ADD_REQUIRED_DEPENDENCY("hrp2-dynamics >= 1.0.1")
ADD_REQUIRED_DEPENDENCY("fast-replanning-data >= 1.0.0")

SET(BOOST_COMPONENTS
  system thread filesystem
)
SEARCH_FOR_BOOST()

FIND_PACKAGE(Boost)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
set(Boost_LIBRARIES
	${Boost_LIBRARIES}
	/usr/lib/libboost_system.so
	/usr/lib/libboost_thread.so
)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
	message(STATUS ${Boost_LIBRARIES})
	message(STATUS ${Boost_INCLUDE_DIRS})
	message(STATUS ${Boost_LIBRARY_DIRS})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

include(CheckFunctionExists)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#include_directories("/opt/ros/electric/stacks/visualization/interactive_markers")

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
	set(LibFann_LIBRARIES
		fann
		m
	)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -lfann")
else()
	message(STATUS "Compilation without fann libary")
endif()
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
	add_definitions("-DEIGEN_FOUND")
	message(STATUS "------------------- ")
else()
	message(STATUS "Compilation without EIGEN libary")
endif()
##########################################################
##########################################################

# Configure robot path to hrp-2 14 
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/src/planner/robot_path.cpp.in	
	${CMAKE_CURRENT_BINARY_DIR}/src/robot_path.cpp)
#ADD_CUSTOM_TARGET( ros rosmake ${PROJECT_NAME}
#COMMAND rosrun ${PROJECT_NAME} Main 
#COMMENT "rosmake & rosrun"
#)
set(SOURCE
	src/environment/environment.h
	src/environment/environment.cpp
	src/environment/sallebauzil.cpp
	src/environment/13humanoids.cpp
	src/environment/13humanoids_real.cpp
	src/environment/testbed.cpp
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
	src/planner/trajectory_visualizer.h 
	src/planner/trajectory_visualizer.cpp
	src/planner/motion_generator.h
	src/planner/motion_generator.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/src/robot_path.cpp
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
	src/genFullBodyTrajectory.cpp
	src/genFullBodyTrajectory.h
)

add_definitions("-std=c++0x")
include_directories(src)
set(UTIL_LIB util)
rosbuild_add_library(${UTIL_LIB} ${SOURCE})
target_link_libraries(${UTIL_LIB} ${FCLLIB_LIBRARIES})
target_link_libraries(${UTIL_LIB} ${Boost_LIBRARIES})

################################################################################
###### MAIN PROJECTS #####################
################################################################################
#rosbuild_add_executable(${MAIN_EXE} src/main.cpp )
#target_link_libraries(${MAIN_EXE} ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
################################################################################
##### PROJECTS #####################
################################################################################
#rosbuild_add_executable(PerrinPlannerReImplementation test/perrin_reimplementation.cpp )
#target_link_libraries(PerrinPlannerReImplementation ${FPlanner_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})

#rosbuild_add_executable(contactPDF test/contact_pdf_creator.cpp )
#PKG_CONFIG_USE_DEPENDENCY(contactPDF jrl-mal)
#PKG_CONFIG_USE_DEPENDENCY(contactPDF analyticalPG)
#target_link_libraries(contactPDF ${UTIL_LIB} ${FCLLIB_LIBRARIES} ${PQP_LDFLAGS})
rosbuild_add_executable(OrtheyFastRePlanner test/orthey_fastplanner.cpp )
PKG_CONFIG_USE_DEPENDENCY(OrtheyFastRePlanner jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(OrtheyFastRePlanner analyticalPG)
PKG_CONFIG_USE_DEPENDENCY(OrtheyFastRePlanner hrp2-dynamics)
target_link_libraries(OrtheyFastRePlanner ${UTIL_LIB} ${FCLLIB_LIBRARIES} ${PQP_LDFLAGS})

rosbuild_add_executable(OrtheySVDrawer test/orthey_fastplanner_draw_sv.cpp )
PKG_CONFIG_USE_DEPENDENCY(OrtheySVDrawer jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(OrtheySVDrawer analyticalPG)
PKG_CONFIG_USE_DEPENDENCY(OrtheySVDrawer hrp2-dynamics)
target_link_libraries(OrtheySVDrawer ${UTIL_LIB} ${FCLLIB_LIBRARIES} ${PQP_LDFLAGS})

rosbuild_add_executable(FastRePlanner test/perrin_fastplanner.cpp )
PKG_CONFIG_USE_DEPENDENCY(FastRePlanner jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(FastRePlanner analyticalPG)
PKG_CONFIG_USE_DEPENDENCY(FastRePlanner hrp2-dynamics)
target_link_libraries(FastRePlanner ${UTIL_LIB} ${FCLLIB_LIBRARIES} ${PQP_LDFLAGS})

#SET_TARGET_PROPERTIES(FastRePlanner PROPERTIES LINK_FLAGS -lboost_system-mt)
#TARGET_LINK_LIBRARIES(FastRePlanner ${Boost_LIBRARIES})

#rosbuild_add_executable(PublishHRP2 test/publish_hrp2.cpp )
#target_link_libraries(PublishHRP2 ${PQP_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})

#rosbuild_add_executable(PlaybackSteps test/perrin_fastplanner_playback.cpp )
#target_link_libraries(PlaybackSteps ${PQP_LIBRARIES} ${UTIL_LIB} ${FCLLIB_LIBRARIES})
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
