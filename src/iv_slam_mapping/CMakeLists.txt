cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
project(iv_slam_mapping)
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to debug") 
add_definitions(-w)

set(IV_SLAM_MAPPING_MAJOR_VERSION 1)
set(IV_SLAM_MAPPING_MINOR_VERSION 0)
set(IV_SLAM_MAPPING_PATCH_VERSION 0)
set(IV_SLAM_MAPPING_VERSION ${IV_SLAM_MAPPING_MAJOR_VERSION}.${IV_SLAM_MAPPING_MINOR_VERSION}.${IV_SLAM_MAPPING_PATCH_VERSION})
set(IV_SLAM_MAPPING_SOVERSION ${IV_SLAM_MAPPING_MAJOR_VERSION}.${IV_SLAM_MAPPING_MINOR_VERSION})

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_iv_slam_mapping_project()
#google_enable_testing()
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS thread filesystem system iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Protobuf REQUIRED)
find_package(octomap 1.8 REQUIRED)#LZZ
# find_package(OpenCV 2 REQUIRED)
# find_package(OpenCV 2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS  roscpp  sensor_driver_msgs iv_slam_ros_msgs image_transport cv_bridge nav_msgs)
find_package(ivcommon REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_driver_msgs  iv_slam_ros_msgs image_transport cv_bridge nav_msgs
  # DEPENDS OpenCV
)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

# Only build the documentation if we can find Sphinx.
# find_package(Sphinx)
# if(SPHINX_FOUND)
#   add_subdirectory("docs")
# endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/iv_slam_mapping)

# set(IV_SLAM_MAPPING_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/iv_slam_mapping/configuration_files
#   CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/iv_slam_mapping/)
# install(DIRECTORY mapdata DESTINATION share/iv_slam_mapping_ros/mapdata/map/)#lzz
install(DIRECTORY cmake DESTINATION share/iv_slam_mapping/)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")

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
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} -I ${ivcommon_PROTO_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

add_library(${PROJECT_NAME} ${ALL_SRCS})

# configure_file(
#   ${PROJECT_SOURCE_DIR}/iv_slam_mapping/common/config.h.cmake
#   ${PROJECT_BINARY_DIR}/iv_slam_mapping/common/config.h)



#foreach(ABS_FIL ${ALL_TESTS})
 # file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  #get_filename_component(DIR ${REL_FIL} DIRECTORY)
 # get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  # Replace slashes as required for CMP0037.
 # string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
 # google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
#endforeach()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}" "${catkin_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${OPENCV_INCLUDE_DIR}")#LZZ
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})#LZZ

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${OCTOMAP_INCLUDE_DIRS}")#LZZ
target_link_libraries(${PROJECT_NAME} PUBLIC ${OCTOMAP_LIBRARIES})#LZZ

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# We expect find_package(Ceres) to have located these for us.
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CAIRO_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} pthread)

message("ivcommon_INCLUDE_DIRS: ${ivcommon_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${ivcommon_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${ivcommon_LIBRARIES})


# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# TODO(damonkohler): Create a testing library.
#target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  #"${GMOCK_INCLUDE_DIRS}")
#target_link_libraries(${PROJECT_NAME} PUBLIC ${GMOCK_LIBRARY})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT IvslammappingExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

set(IV_SLAM_MAPPING_CMAKE_DIR share/iv_slam_mapping/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  iv_slam_mapping-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/iv_slam_mapping/iv_slam_mapping-config.cmake
  PATH_VARS IV_SLAM_MAPPING_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/iv_slam_mapping
)

install(
  EXPORT IvslammappingExport
  DESTINATION share/iv_slam_mapping/cmake/
  FILE Iv_slam_mappingTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/iv_slam_mapping/iv_slam_mapping-config.cmake
  DESTINATION share/iv_slam_mapping/
)
