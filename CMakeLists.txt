cmake_minimum_required(VERSION 2.8.3)

project(motion_force_control)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
)

# add_message_files(
#    FILES
#    # FootMouseMsg.msg
# )


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/unifiedMotionForceController_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation kdl_parser dynamic_reconfigure
  # DEPENDS Eigen
 )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(unifiedMotionForceController src/unifiedMotionForceController_node.cpp src/UnifiedMotionForceController.cpp)
target_link_libraries(unifiedMotionForceController ${catkin_LIBRARIES})
add_dependencies(unifiedMotionForceController ${PROJECT_NAME}_gencpp)

add_executable(motionController src/motionController_node.cpp src/MotionController.cpp)
target_link_libraries(motionController ${catkin_LIBRARIES})
add_dependencies(motionController ${PROJECT_NAME}_gencpp)