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

add_definitions (-D DEBUG)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

#common commands for building c++ executables and libraries
rosbuild_add_executable(trajectory_planner_node 
    src/TrajectoryAdapter.cpp
    src/TrajectoryAdapterObserverDummy.cpp
    src/TrajectoryAdapterNode.cpp 
    src/PathPlannerRos.cpp
    src/ChaikinCurveApproximation.cpp
    src/DouglasPeuckerApproximation.cpp
    src/Stopwatch.cpp
    src/ChaikinCurveApproximation.cpp
    src/CollisionCheckingRos.cpp
    src/PathIterator.cpp
    src/LinearInterpolation.cpp
    src/OmniDrive.cpp
    src/DifferentialDrive.cpp
)


