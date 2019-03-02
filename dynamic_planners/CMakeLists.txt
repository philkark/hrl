cmake_minimum_required(VERSION 2.8.3)
project(dynamic_planners)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(gazebo REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${OpenMP_C_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS} ${GAZEBO_CXX_FLAGS} -DOPENMP_MAP -march=native -mtune=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -DOPENMP_MAP -march=native -mtune=native")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native -fstack-protector -fstack-protector-all")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${GAZEBO_CXX_FLAGS} -march=native -mtune=native")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  visualization_msgs
  trajectory_msgs
  kdl_parser
  math_std
  tools_std
  tf
  message_generation
  reemc_trajectory_execution
)

find_package(orocos_kdl REQUIRED)

add_message_files(FILES
  gazebo_object.msg
)

add_service_files(FILES
  get_height_map.srv
)

generate_messages()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES dynamic_height_map dynamic_astar_grid_planner gazebo_plugin_reemc_rviz_simulation
   CATKIN_DEPENDS roscpp roslib std_msgs visualization_msgs trajectory_msgs kdl_parser math_std tools_std tf message_runtime reemc_trajectory_execution
   DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(SRC_FILES_COMMON
  src/common/progress_publisher.cpp
)

set(SRC_FILES_MAP
  src/lib_dynamic_height_map/dynamic_height_map.cpp
  src/lib_dynamic_height_map/dynamic_height_map_config.cpp
  src/lib_dynamic_height_map/objects.cpp
  src/lib_dynamic_height_map/segment.cpp
  src/lib_dynamic_height_map/simulation.cpp
  src/lib_dynamic_height_map/cells.cpp
  src/lib_dynamic_height_map/global_map.cpp
)

set(SRC_FILES_ASTAR
  src/lib_dynamic_astar_grid_planner/dynamic_astar_grid_planner.cpp
  src/lib_dynamic_astar_grid_planner/dynamic_astar_grid_planner_structs.cpp
)

set(SRC_FILES_FOOTSTEP
  src/lib_dynamic_footstep_planner/expansion_map.cpp
  src/lib_dynamic_footstep_planner/dynamic_footstep_planner.cpp
  src/lib_dynamic_footstep_planner/dynamic_footstep_planner_config.cpp
  src/lib_dynamic_footstep_planner/dynamic_footstep_planner_structs.cpp
)

add_library(dynamic_planners_common ${SRC_FILES_COMMON})
target_link_libraries(dynamic_planners_common ${catkin_LIBRARIES})

add_library(gazebo_plugin_reemc_rviz_simulation  SHARED src/gazebo_plugins/gazebo_plugin_reemc_rviz_simulation.cpp)
target_link_libraries(gazebo_plugin_reemc_rviz_simulation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_plugin_get_height_map_service  SHARED src/gazebo_plugins/gazebo_plugin_get_height_map_service.cpp)
add_dependencies(gazebo_plugin_get_height_map_service ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(gazebo_plugin_get_height_map_service ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(dynamic_height_map ${SRC_FILES_MAP})
add_dependencies(dynamic_height_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_height_map ${catkin_LIBRARIES} ${OpenMP_LIBS} ${orocos_kdl_LIBRARIES} gazebo_plugin_get_height_map_service dynamic_planners_common)

add_library(dynamic_astar_grid_planner ${SRC_FILES_ASTAR})
target_link_libraries(dynamic_astar_grid_planner ${catkin_LIBRARIES} dynamic_height_map dynamic_planners_common)

add_library(dynamic_footstep_planner ${SRC_FILES_FOOTSTEP})
target_link_libraries(dynamic_footstep_planner ${catkin_LIBRARIES} dynamic_height_map dynamic_planners_common)

add_executable(dynamic_rviz_controller src/dynamic_rviz_controller.cpp src/dynamic_rviz_visualizer.cpp)
target_link_libraries(dynamic_rviz_controller ${catkin_LIBRARIES} dynamic_height_map dynamic_astar_grid_planner dynamic_footstep_planner dynamic_planners_common) 


