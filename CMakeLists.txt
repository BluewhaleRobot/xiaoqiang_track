cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_track)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CURL REQUIRED libcurl)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLIDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

add_message_files(
  FILES
  TrackTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation cv_bridge message_runtime
)

add_executable(tracking_node
  ${PROJECT_SOURCE_DIR}/src/tracking_node.cpp
  ${PROJECT_SOURCE_DIR}/src/baidu_track.cpp
  ${PROJECT_SOURCE_DIR}/src/tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/body_pose_track.cpp
)

target_link_libraries(tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${JSONCPP_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  ${CURL_LIBRARIES}
)
