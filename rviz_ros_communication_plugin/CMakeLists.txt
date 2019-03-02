cmake_minimum_required(VERSION 2.8.3)
project(rviz_ros_communication_plugin)

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
  rviz
  roscpp
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rviz_ros_communication_plugin
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/ros_communication_panel.cpp
  src/slider.cpp
  src/category.cpp
  src/text_field.cpp
  src/button.cpp
  src/info_text.cpp
)

set(HEADER_FILES
   include/rviz_ros_communication_plugin/ros_communication_panel.h
   include/rviz_ros_communication_plugin/slider.h
   include/rviz_ros_communication_plugin/category.h
   include/rviz_ros_communication_plugin/text_field.h
   include/rviz_ros_communication_plugin/button.h
   include/rviz_ros_communication_plugin/info_text.h
)

add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  reemc_rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
