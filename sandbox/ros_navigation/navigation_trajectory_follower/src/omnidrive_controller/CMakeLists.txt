FILE(GLOB_RECURSE OMNIDRIVE_CONTROLLER_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp" )
rosbuild_add_library(omnidrive_controller ${OMNIDRIVE_CONTROLLER_SOURCES})
#rosbuild_add_executable(20_sim_cartesian_compliance_control_test  ${20SIM_GERNERATED_SOURCES})

