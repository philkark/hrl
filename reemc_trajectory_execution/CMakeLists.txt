cmake_minimum_required(VERSION 2.8.3)
project(reemc_trajectory_execution)

#check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  math_std
  tools_std
  roscpp
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  geometry_msgs
  std_msgs
  kdl_parser
  tf
  message_generation
)

find_package(orocos_kdl REQUIRED)

add_service_files(FILES
  single_footstep_execution.srv
  step_to_init_pose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS
    include
# LIBRARIES reemc_trajectory_execution
  CATKIN_DEPENDS
    math_std 
    tools_std 
    roscpp 
    actionlib 
    control_msgs
    trajectory_msgs
    sensor_msgs 
    geometry_msgs
    std_msgs 
    kdl_parser 
    tf 
    message_generation
  DEPENDS
    orocos_kdl 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(footstep_execution_node
               src/main.cpp 
               src/robot_trajectory_generator.cpp 
               src/ellipse_trajectory_generator.cpp
               src/rectangular_trajectory_generator.cpp
               src/linear_trajectory_generator.cpp 
               src/trajectory_generator.cpp 
               src/pose6d.cpp
               src/footstep_executor.cpp)
add_dependencies(footstep_execution_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(footstep_execution_node ${catkin_LIBRARIES})
