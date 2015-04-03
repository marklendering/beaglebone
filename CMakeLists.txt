cmake_minimum_required(VERSION 2.8.3)
project(beaglebone)

find_package(catkin REQUIRED COMPONENTS 
	roscpp rospy rostest std_msgs tf tf2
	message_generation geometry_msgs sensor_msgs
)
find_package(Boost REQUIRED)


add_message_files(
  DIRECTORY
  msg
  FILES
  IMU.msg
  LRS.msg
  WheelVelocities.msg
  WheelDistances.msg
)

#add_service_files(
#  FILES
#)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



#sensor and motor drivers
add_executable(MotionSensorDriver src/demconrobot_MotionSensorDriver.cpp)
add_library(Driver src/MotionSensorDriver.cpp src/CanDriver.cpp)
target_link_libraries(MotionSensorDriver Driver)
target_link_libraries(MotionSensorDriver ${catkin_LIBRARIES})
target_link_libraries(MotionSensorDriver ${Boost_LIBRARIES})
add_dependencies(MotionSensorDriver beaglebone_gencpp)
add_dependencies(Driver beaglebone_gencpp)

#robot controller

add_executable(RobotController src/demconrobot_player.cpp src/player_node.cpp) 
#add_dependencies(RobotController package_name_generate_messages_cpp)
target_link_libraries(RobotController ${catkin_LIBRARIES})
target_link_libraries(RobotController ${Boost_LIBRARIES})
add_dependencies(RobotController beablebone_gencpp)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beaglebone.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
