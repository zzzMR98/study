cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation
nav_msgs
geometry_msgs
sensor_msgs
roslib
rospy
tf
control_msgs
sensor_driver_msgs
covgrid_slam_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ##GpswithHeading.msg
  ##OdometrywithGps.msg
  ##gpsmsg.msg
  ##InsVelocity.msg
  ##ECUData.msg
  ##GetECUReport.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  nav_msgs
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(Eigen3 REQUIRED)




find_package(ivcommon REQUIRED)
LIST( APPEND CMAKE_MODULE_PATH ${IVCOMMON_CMAKE_DIR} )
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)




catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
  DEPENDS EIGEN3 
  INCLUDE_DIRS src
#  LIBRARIES sensor_fusion
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${ivcommon_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} src include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sensor_fusion.cpp
# )
set(Analysis_SRC src/include/AnalysisGPS.cpp src/include/pose_extrapolator.cc)#同理define
set(util_SRC src/include/util/utm/utm.cpp src/include/util/utm/datum.cpp src/include/util/xmlconf/xmlconf.cpp src/include/util/xmlconf/tinyxml2.cpp)
add_library(Analysis SHARED ${Analysis_SRC})#编译链接库，SHARED：动态链接库 ivsensor：随便起的要生成的动态链接库的名字
add_library(UTIL_LIBS SHARED ${util_SRC})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sensor_fusion_node.cpp)
add_executable(lidar_filter src/lidar_filter.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

#add_dependencies(lidar_filter sensor_fusion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(Analysis ${catkin_LIBRARIES} UTIL_LIBS)

target_link_libraries(lidar_filter ${catkin_LIBRARIES} Analysis ${ivcommon_LIBRARIES} ${Boost_LIBRARIES} UTIL_LIBS glog gflags)#hello这个程序编译时需要的库，包括动态库


set(PROJECTS lidar_filter UTIL_LIBS Analysis)

#using c++11
set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties( ${PROJECTS} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(DIRECTORY launch DESTINATION share/sensor_fusion/)

install(TARGETS ${PROJECTS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECTS}
  EXPORT sensor_fusionExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
