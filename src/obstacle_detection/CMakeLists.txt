cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  rosbag
  roscpp
  roslib
  sensor_msgs
  negative_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(ivcommon REQUIRED)

find_package(sensor_driver REQUIRED)
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

install(DIRECTORY launch DESTINATION share/obstacle_detection/)
install(DIRECTORY rviz_cfg DESTINATION share/obstacle_detection/)

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)
#find_package(PCL REQUIRED)
find_package(PCL REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
# find_package(OpenCV REQUIRED)
# find_package(OpenCV 2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(sensor_driver REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} ${ivcommon_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} src include)


add_executable(back_obstacle_node src/back_obstacle_node.cpp src/obstacle_detection.cpp)

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp src/obstacle_detection.cpp)
target_link_libraries(obstacle_detection_node PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS}
  utils
  ivsensor glog gflags )
target_link_libraries(back_obstacle_node PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS}
  utils
  ivsensor glog gflags )
# set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
# set_target_properties(obstacle_detection_node back_obstacle_node PROPERTIES
#   COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS obstacle_detection_node back_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS obstacle_detection_node back_obstacle_node
  EXPORT obstacle_detection_nodeExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
