cmake_minimum_required(VERSION 2.8.3)
project(slopeogm_msgs)

set(PACKAGE_DEPENDENCIES
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  ${PACKAGE_DEPENDENCIES}
)

add_message_files(
  FILES
  SlopeOGM.msg
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    # ${PACKAGE_DEPENDENCIES}
    message_runtime
)
