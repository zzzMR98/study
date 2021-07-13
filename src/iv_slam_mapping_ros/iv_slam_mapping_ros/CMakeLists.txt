
cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

add_compile_options(-Wno-unused-result -Wno-format -Wno-sign-compare)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

project(iv_slam_mapping_ros)
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type")  
set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
  iv_dynamicobject_msgs
  iv_slam_ros_msgs
  covgrid_slam_msgs
  sensor_driver_msgs
  compressed_pointcloud_msgs
  # iv_slam_mapping
  # ivcommon
#grid_map_msgs#lzz
#grid_map_core#lzz
#grid_map_octomap#lzz
#grid_map_ros
)

find_package(ivcommon REQUIRED)
# find_package(OpenCV 2 REQUIRED)
find_package(OpenCV 3 REQUIRED)

# find_package(tbb_common REQUIRED)
#include("${IV_SLAM_MAPPING_CMAKE_DIR}/functions.cmake")
#set(BUILD_SHARED_LIBS OFF)
#google_initialize_iv_slam_mapping_project()
#google_enable_testing()

find_package(iv_slam_mapping REQUIRED)
include("${IV_SLAM_MAPPING_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_iv_slam_mapping_project()

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Fix for PCL linking issue in Zesty, as in
# https://github.com/ros-perception/perception_pcl/blob/b53b5d5c3abc7c3afde8a77177674e6d74c59522/pcl_ros/CMakeLists.txt#L12
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # FIXME: this causes duplicates and not found error in ubuntu:zesty
  list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")
endif()
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
  ${ivcommon_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
#set(GTEST_FOUND TRUE)
#set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
#set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
add_library(${PROJECT_NAME} ${ALL_SRCS})
add_subdirectory("iv_slam_mapping_ros")

target_link_libraries(
  ${PROJECT_NAME} PUBLIC 
  # ${iv_slam_mapping_LIBRARIES}
  iv_slam_mapping
  ${ivcommon_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  tbb_common
)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})



install(DIRECTORY launch urdf slam_configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_remove_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install source headers.
file(GLOB_RECURSE HDRS "*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()
