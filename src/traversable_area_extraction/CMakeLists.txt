cmake_minimum_required(VERSION 2.8.3)
project(traversable_area_extraction)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(PACKAGE_DEPENDENCIES  
  geometry_msgs
  sensor_msgs
  roscpp
  roslib
  std_msgs
  # cv_bridge
  iv_slam_ros_msgs
  covgrid_slam_msgs
  obstacle_msgs
  slopeogm_msgs
  stiff_msgs
  uneven_area_msgs
  eigen_conversions
  message_runtime
  message_filters
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})


#message("TBB_COMMON_CMAKE_DIR:" ${TBB_COMMON_CMAKE_DIR} )
#include("${TBB_COMMON_CMAKE_DIR}/functions.cmake")
#set(BUILD_SHARED_LIBS OFF)
#google_initialize_tbb_common_project()
find_package(ivcommon REQUIRED)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_ivcommon_project()
include(FindPkgConfig)
find_package(Eigen3 REQUIRED)
# find_package(OpenCV 2.4.13 REQUIRED)
# find_package(OpenCV 2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams thread)
find_package(LuaGoogle REQUIRED)
# message("LuaGoogle dir: ${LuaGoogle_INCLUDE_DIR}")
# find_package(Lua REQUIRED)
# message("lua dir: ${LUA_INCLUDE_DIR}")
# include_directories(
# 	${catkin_INCLUDE_DIRS}
# 	${EIGEN3_INCLUDE_DIR}
#   ${Boost_INCLUDE_DIRS}
#   ${ivcommon_INCLUDE_DIRS}
#   ${LUA_INCLUDE_DIR}
#   ${OpenCV_INCLUDE_DIRS}
# )
catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  # DEPENDS EIGEN3 OpenCV Boost
)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h" )

file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
# message( " ${ALL_SRCS}" ${ALL_SRCS})
add_executable(traversable_area_extraction_main traversable_area_extraction/traversable_area_extraction_main.cc ${ALL_SRCS})

message("Boost_FOUND: ${Boost_FOUND}")
message("Boost_VERSION: ${Boost_VERSION}")
message("Boost_LIBRARIES: ${Boost_LIBRARIES}")
message("Boost_LIBRARIES: ${Boost_LIBS}")

target_include_directories(traversable_area_extraction_main SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${ivcommon_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(traversable_area_extraction_main PUBLIC 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBS}
  ${ivcommon_LIBRARIES} glog gflags
)
# opencv
# target_include_directories(traversable_area_extraction_main SYSTEM PUBLIC
#   "${OpenCV_INCLUDE_DIRS}")
# target_link_libraries(traversable_area_extraction_main PUBLIC ${OpenCV_LIBS})

# eigen
# target_include_directories(traversable_area_extraction_main SYSTEM PUBLIC
#   "${EIGEN3_INCLUDE_DIR}")
# target_link_libraries(traversable_area_extraction_main PUBLIC ${EIGEN3_LIBRARIES})

# Boost
# target_include_directories(traversable_area_extraction_main SYSTEM PUBLIC
#   "${Boost_INCLUDE_DIRS}")
# target_link_libraries(traversable_area_extraction_main PUBLIC ${Boost_LIBRARIES})

add_executable(test_traversable_area_node test/test_traversable_area_node.cpp)
add_executable(visualize_traversable_area_node test/visualize_traversable_area_node.cpp)

target_include_directories(test_traversable_area_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(visualize_traversable_area_node SYSTEM PUBLIC 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ivcommon_INCLUDE_DIRS}
)
target_link_libraries(test_traversable_area_node
  ${catkin_LIBRARIES}
)
target_link_libraries(visualize_traversable_area_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${ivcommon_LIBRARIES}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS traversable_area_extraction_main 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS test_traversable_area_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS visualize_traversable_area_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
