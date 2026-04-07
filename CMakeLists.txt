cmake_minimum_required(VERSION 3.5)
project(mrs_multirotor_simulator)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# export command for Clang
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  pluginlib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  mrs_uav_hw_api
  Eigen3
  backward_ros
  #HwApiMrsFcu dependencies
  mrs_modules_msgs
  mavros_msgs
)

set(LIBRARIES
  uMsgLib
  MrsMultirotorSimulator_HwApiMrsFcu
  MrsMultirotorSimulator_FcuHitlBinder
  MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
)

set(UMSG_PATH ${CMAKE_CURRENT_DIR}umsg/umsg_lib/umsg_lib)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  include/mrs_multirotor_simulator
  include/mrs_multirotor_simulator/uav_system
  include/mrs_multirotor_simulator/uav_system/controllers
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_uav_hw_api_INCLUDE_DIRS}
  ${UMSG_PATH}/core/inc
  ${UMSG_PATH}/inc
)

# ROS Wrapper for the Multirotor UAV System + controllers

add_library(MrsMultirotorSimulator_SimulatorSystemRos SHARED
  src/uav_system_ros.cpp
  #src/iir_filter.cpp
)

ament_target_dependencies(MrsMultirotorSimulator_SimulatorSystemRos
  ${DEPENDENCIES}
)

# MRS Simulator

add_library(MrsMultirotorSimulator_Simulator SHARED
  src/multirotor_simulator.cpp
)

target_link_libraries(MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
  )

ament_target_dependencies(MrsMultirotorSimulator_Simulator
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsMultirotorSimulator_Simulator PLUGIN "mrs_multirotor_simulator::MultirotorSimulator" EXECUTABLE MrsMultirotorSimulator_Simulator)


#Generate uMsgLib source code
execute_process(
    COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/generate_umsg_library.sh"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE result
    ERROR_VARIABLE error
)

if(NOT result EQUAL 0)
    message(FATAL_ERROR "Script generate_umsg_library.sh failed with error: ${error}")
endif()

# UMSG Library

#set(UMSG_PORT "POSIX")

add_library(uMsgLib SHARED
    ${UMSG_PATH}/core/src/umsg.c
    # generated files
    ${UMSG_PATH}/src/umsg_sensors.c
    ${UMSG_PATH}/src/umsg_estimation.c
    ${UMSG_PATH}/src/umsg_offboard.c
    ${UMSG_PATH}/src/umsg_autonomy.c
    ${UMSG_PATH}/src/umsg_state.c
    ${UMSG_PATH}/src/umsg_debug.c
    ${UMSG_PATH}/src/umsg_control.c
    ${UMSG_PATH}/src/umsg_classes.c
  )

#Select POSIX port
target_sources(uMsgLib PRIVATE ${UMSG_PATH}/core/src/port_posix.c)

ament_target_dependencies(uMsgLib
  ${DEPENDENCIES}
)

# HW API MRS FCU
# Plugin library for interfacing Simulator with FCU HW

add_library(MrsMultirotorSimulator_HwApiMrsFcu SHARED
  src/hw_api_mrs_fcu.cpp
  src/serial_port.cpp
  src/serial_api.cpp
  )

target_link_libraries(MrsMultirotorSimulator_HwApiMrsFcu
  uMsgLib
  )

ament_target_dependencies(MrsMultirotorSimulator_HwApiMrsFcu
  ${DEPENDENCIES}
)

target_compile_definitions(MrsMultirotorSimulator_HwApiMrsFcu PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# FCU HITL Binder
# Interfacing FCU with HITL simulation


add_library(MrsMultirotorSimulator_FcuHitlBinder SHARED
  src/fcu_hitl_binder.cpp
  src/serial_port.cpp
  )

target_link_libraries(MrsMultirotorSimulator_FcuHitlBinder
  uMsgLib
  )

ament_target_dependencies(MrsMultirotorSimulator_FcuHitlBinder
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsMultirotorSimulator_FcuHitlBinder PLUGIN "mrs_hitl_binders::FcuHitlBinder" EXECUTABLE MrsMultirotorSimulator_FcuHitlBinder)


## HW Api Plugin
#
#add_library(MrsMultirotorSimulator_HwApiPlugin SHARED
#  src/hw_api_plugin.cpp
#)
#
#ament_target_dependencies(MrsMultirotorSimulator_HwApiPlugin
#  ${DEPENDENCIES}
#)
#
#target_compile_definitions(MrsMultirotorSimulator_HwApiPlugin PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
#ament_export_libraries(MrsMultirotorSimulator_HwApiMrsFcu)
#pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

# Export the HwApi library for pluginlib
ament_export_libraries(MrsMultirotorSimulator_HwApiMrsFcu)

# Install and register our plugins.xml (ensure plugins.xml is in package root)
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Tell pluginlib / ament where our plugin XML is (use this package name)
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()


# -----------------------------
# | Test plugin loader executable
# -----------------------------
add_executable(test_plugin test/test_plugin.cpp)
ament_target_dependencies(test_plugin
  pluginlib
  mrs_uav_hw_api
  rclcpp
)
target_include_directories(test_plugin PUBLIC
  include
  ${mrs_uav_hw_api_INCLUDE_DIRS}
)

install(TARGETS test_plugin
  DESTINATION lib/${PROJECT_NAME}
)