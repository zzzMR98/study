cmake_minimum_required(VERSION 2.8.3)
project(covgrid_slam_ros)
#add_compile_options(-Wno-narrowing)

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

LIST( APPEND CMAKE_MODULE_PATH ${IVCOMMON_CMAKE_DIR} )
find_package(sensor_driver REQUIRED)
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()


include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
# find_package(OpenCV REQUIRED)
# find_package(OpenCV 2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(Protobuf REQUIRED)

find_package(Ceres REQUIRED )
#g2o
find_package( G2O REQUIRED )

find_package( CSparse REQUIRED )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse
        g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension
        g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser
        g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse  g2o_solver_cholmod cholmod)


set(COVGRID_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/covgrid_slam_ros/config
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY config DESTINATION share/covgrid_slam_ros/)
install(DIRECTORY urdf DESTINATION share/covgrid_slam_ros/)
install(DIRECTORY launch DESTINATION share/covgrid_slam_ros/)
install(DIRECTORY rviz_cfg DESTINATION share/covgrid_slam_ros/)

set(LINKER_LIBS "")
list(APPEND LINKER_LIBS ${CERES_LIBRARIES})

find_package(Glog REQUIRED)

list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})

find_package(GFlags REQUIRED)

list(APPEND LINKER_LIBS ${GFLAGS_LIBRARIES})


list(APPEND LINKER_LIBS ${LUA_LIBRARIES})


list(APPEND LINKER_LIBS ${PROTOBUF_LIBRARY})

list(APPEND LINKER_LIBS ${CERES_LIBRARIES} ${LUA_LIBRARIES} ${PROTOBUF_LIBRARY})

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS covgrid_slam
)
# message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!${senserdriver_INCLUDE_DIRS}")
include_directories(SYSTEM covgrid_slam ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ivcommon_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${senserdriver_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SLAM_SRCS "covgrid_slam/mapping/*.cc" "covgrid_slam/mapping/*.h" "covgrid_slam/mapping3d/*.cc" "covgrid_slam/mapping3d/*.h"  "covgrid_slam/sensor/*.cc" "covgrid_slam/sensor/*.h" "covgrid_slam/common/*.cc" "covgrid_slam/common/*.h")


file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})

  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    PRE_BUILD
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR}  ${ABS_FIL} -I ${ivcommon_PROTO_DIR}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()




set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_SLAM_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

add_library(covgridslam SHARED ${ALL_SLAM_SRCS} )


target_link_libraries(covgridslam  PUBLIC   ${OpenCV_LIBS} ${LINKER_LIBS} ${G2O_LIBS} ${ivcommon_LIBRARIES})




target_include_directories(covgridslam  SYSTEM  PUBLIC "${CSPARSE_INCLUDE_DIR}" )
target_include_directories(covgridslam  SYSTEM  PUBLIC "${CERES_INCLUDE_DIRS}")
target_include_directories(covgridslam  SYSTEM  PUBLIC  "${GLOG_INCLUDE_DIRS}")
target_include_directories(covgridslam  SYSTEM  PUBLIC  "${GFLAGS_INCLUDE_DIRS}")
target_include_directories(covgridslam  SYSTEM  PUBLIC  "${LUA_INCLUDE_DIR}")
target_include_directories(covgridslam  SYSTEM  PUBLIC "${PCL_INCLUDE_DIRS}")

add_executable(lidarodometrymain covgrid_slam/lidarodometrymain.cpp)

target_link_libraries(lidarodometrymain PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS}  utils covgridslam ${LINKER_LIBS} ${senserdriver_LIBRARIES} )

add_executable(preciseodometrymain covgrid_slam/preciseodometrymain.cpp)

target_link_libraries(preciseodometrymain PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} utils covgridslam ${LINKER_LIBS} ${senserdriver_LIBRARIES} )

add_executable(transformmain covgrid_slam/transformmain.cpp)
target_link_libraries(transformmain PUBLIC ${catkin_LIBRARIES} ${ivcommon_LIBRARIES}   ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} ${sensordriver_LIBRARIES} utils covgridslam ivsensor glog gflags )

add_executable(converttoearth covgrid_slam/converttoearth.cpp )
target_link_libraries(converttoearth PUBLIC ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} ${sensordriver_LIBRARIES} utils ivcommon ivsensor glog gflags )

add_executable(multimapinteg covgrid_slam/multimapinteg.cpp)

target_link_libraries(multimapinteg PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS}  utils covgridslam ${LINKER_LIBS} ${senserdriver_LIBRARIES} )

add_executable(INS_LIDAR_calibration covgrid_slam/INS_LIDAR_calibration.cpp)

target_link_libraries(INS_LIDAR_calibration PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS}  utils covgridslam ${LINKER_LIBS} ${senserdriver_LIBRARIES} )

set(PROJECTS lidarodometrymain transformmain converttoearth preciseodometrymain multimapinteg INS_LIDAR_calibration)
include_directories(covgridslam PUBLIC ${PROJECT_BINARY_DIR}/covgrid_slam ${PROJECT_SOURCE_DIR}/covgrid_slam ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

target_include_directories(covgridslam PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties( ${PROJECTS} covgridslam PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# Install source headers.
file(GLOB_RECURSE hdrs "*.h")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)

  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${PROJECT_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

set(COVGRIDSLAM_CMAKE_DIR share/covgrid_slam_ros/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  covgrid_slam_ros-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/covgrid_slam_ros/covgrid_slam_ros-config.cmake
  PATH_VARS COVGRIDSLAM_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/covgrid_slam_ros
)

install(
  TARGETS covgridslam
  EXPORT covgrid_slamExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS ${PROJECTS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECTS}
  EXPORT sensor_driverExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  EXPORT covgrid_slamExport
  DESTINATION share/covgrid_slam_ros/cmake/
  FILE CovgridslamTargets.cmake
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
