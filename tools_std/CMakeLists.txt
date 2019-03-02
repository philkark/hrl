cmake_minimum_required(VERSION 2.8.3)
project(tools_std)

# check c++11 / c++0x
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

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  tf
  math_std
  dynamic_reconfigure
  std_msgs
  nav_msgs
  visualization_msgs
  
)

generate_dynamic_reconfigure_options(
  config/transform6d.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tools_std
   CATKIN_DEPENDS dynamic_reconfigure math_std tf std_msgs nav_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)

add_library(tools_std
  src/ros_param_loader.cpp
  src/timer.cpp
  src/ros_utils.cpp
  src/dynamic_transform_setter.cpp
  src/color_scale.cpp
  src/occupancy_publisher.cpp
)
add_dependencies(tools_std ${catkin_EXPORTED_TARGETS})
target_link_libraries(tools_std ${catkin_LIBRARIES})

install(
  TARGETS tools_std
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
