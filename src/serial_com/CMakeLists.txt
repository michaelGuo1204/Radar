cmake_minimum_required(VERSION 3.0.2)
project(serial_com)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(serial_DIR /tmp/usr/local/share/serial/cmake)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  serial
  std_msgs
)


add_message_files(
   FILES
   comm.msg
)



generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_com
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
  include/serial_com
)


add_executable(stm_com stm_com.cc src/SerialCom.cc)
add_dependencies(stm_com ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stm_com
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
