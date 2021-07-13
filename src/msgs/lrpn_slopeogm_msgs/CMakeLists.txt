cmake_minimum_required(VERSION 2.8.3)
project(lrpn_slopeogm_msgs)

set(PACKAGE_DEPENDENCIES
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
  roscpp
  rospy
  message_generation
)

add_message_files()

generate_messages(
DEPENDENCIES
${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
    # ${PACKAGE_DEPENDENCIES}
)
