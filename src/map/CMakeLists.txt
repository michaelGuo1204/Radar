cmake_minimum_required(VERSION 3.0.2)
project(map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 3 not found.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)
link_directories(
  /opt/MVS/lib/64
  /opt/MVS/lib/32
  /usr/local/lib
#   ${TORCH_LIBRARY_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../serial_com/include
  /opt/MVS/include
  ./include/
  ./include/map
)
aux_source_directory(src/mapcam CAM_SRC)
aux_source_directory(src/mapwork WORK_SRC)
add_executable(Frame Frame.cpp
  ${CAM_SRC}
  ${WORK_SRC}
)
add_dependencies(Frame ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Frame
  MvCameraControl
  libMvCameraControl.so
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

