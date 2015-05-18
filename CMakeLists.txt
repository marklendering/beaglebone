cmake_minimum_required(VERSION 2.8.3)
project(beaglebone)

find_package(catkin REQUIRED COMPONENTS 
	roscpp rospy rostest std_msgs tf tf2
	message_generation geometry_msgs sensor_msgs
)
find_package(Boost REQUIRED)


find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)


#${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${QT_LIBRARIES})


add_executable(ui src/ui.cpp)
target_link_libraries(ui ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

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
