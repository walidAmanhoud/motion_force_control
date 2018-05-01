cmake_minimum_required(VERSION 2.8.3)

project(motion_force_control)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
  foot_interfaces
  svm_grad
  sg_differentiation

)

# add_message_files(
#    FILES
#    # FootMouseMsg.msg
# )

find_package(Eigen3 REQUIRED)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/unifiedMotionForceController_params.cfg 
                                     cfg/motionController_params.cfg
                                     cfg/forceTaskSharedControl_params.cfg
                                     cfg/modulatedDS_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} passive_ds_force_controller
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation kdl_parser dynamic_reconfigure foot_interfaces svm_grad sg_differentiation

  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(unifiedMotionForceController src/unifiedMotionForceController_node.cpp src/UnifiedMotionForceController.cpp)
target_link_libraries(unifiedMotionForceController ${catkin_LIBRARIES} passive_ds_force_controller)
add_dependencies(unifiedMotionForceController ${PROJECT_NAME}_gencpp)

add_executable(motionController src/motionController_node.cpp src/MotionController.cpp)
target_link_libraries(motionController ${catkin_LIBRARIES} passive_ds_force_controller)
add_dependencies(motionController ${PROJECT_NAME}_gencpp)

add_executable(moveToDesiredPose src/moveToDesiredPose_node.cpp src/MoveToDesiredPose.cpp)
target_link_libraries(moveToDesiredPose ${catkin_LIBRARIES})
add_dependencies(moveToDesiredPose ${PROJECT_NAME}_gencpp)

add_executable(forceTaskSharedControl src/forceTaskSharedControl_node.cpp src/ForceTaskSharedControl.cpp)
target_link_libraries(forceTaskSharedControl ${catkin_LIBRARIES})
add_dependencies(forceTaskSharedControl ${PROJECT_NAME}_gencpp)


add_executable(modulatedDS src/modulatedDS_node.cpp src/ModulatedDS.cpp)
target_link_libraries(modulatedDS ${catkin_LIBRARIES})
add_dependencies(modulatedDS ${PROJECT_NAME}_gencpp)

add_executable(surfaceLearning src/surfaceLearning_node.cpp src/SurfaceLearning.cpp)
target_link_libraries(surfaceLearning ${catkin_LIBRARIES})
add_dependencies(surfaceLearning ${PROJECT_NAME}_gencpp)

add_executable(objectGrabbing src/objectGrabbing_node.cpp src/ObjectGrabbing.cpp)
target_link_libraries(objectGrabbing ${catkin_LIBRARIES})
add_dependencies(objectGrabbing ${PROJECT_NAME}_gencpp)

add_executable(twoRobotsTransform src/twoRobotsTransform_node.cpp src/TwoRobotsTransform.cpp)
target_link_libraries(twoRobotsTransform ${catkin_LIBRARIES})
add_dependencies(twoRobotsTransform ${PROJECT_NAME}_gencpp)

#this step is required to have the files how up in qtcreator
# file(GLOB HEADERS "include/*.h")
#file(GLOB HEADERS "include/locally_modulated_ds/*.h")

## Declare a cpp library
add_library(passive_ds_force_controller src/PassiveDsForceController.cpp)

FILE(GLOB SRCS src/UnifiedMotionForceController.cpp src/MotionController.cpp src/MoveToDesiredPose.cpp src/PassiveDsForceController.cpp src/ModulatedDS.cpp)
FILE(GLOB INCS include/UnifiedMotionForceController.h include/MotionController.h include/MoveToDesiredPose.h include/PassiveDsForceController.h include/ModulatedDS.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})