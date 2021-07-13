
cmake_minimum_required(VERSION 2.8.3)

project(iv_slam_ros_msgs)

set(PACKAGE_DEPENDENCIES
  geometry_msgs
  nav_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

# add_message_files(
#   FILES
#     SubmapList.msg
#     SubmapEntry.msg
#     SensorTopics.msg
#     TrajectoryOptions.msg
#     TraversableArea.msg
# insertion_submaps.msg
# SubmapIndex.msg
# OptimizationImu.msg
# OptimizationInsertResult.msg
# Traversablevehiclepose.msg
# PrimarytraversableArea.msg
# )

add_message_files()

add_service_files(
  FILES
    SubmapQuery.srv
    FinishTrajectory.srv
    StartTrajectory.srv
    WriteState.srv
#OptimizationImu.srv
#OptimizationInsertResult.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    # ${PACKAGE_DEPENDENCIES}
)
