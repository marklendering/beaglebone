cmake_minimum_required(VERSION 2.8.3)
project(beaglebone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	std_msgs
	message_generation
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
  CurrentState.msg
  Error.msg
  IMU.msg
  LaserScanB.msg
  LRS.msg
  ModuleState.msg
  OperationState.msg
  Position.msg
  SetMode.msg
  SpeedLR.msg
  Velocity.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AppendError.srv
  Command.srv
  Empty.srv
  Init.srv
  Mode.srv
  SetPosition.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beaglebone
  CATKIN_DEPENDS message_runtime roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(beaglebone
#   src/${PROJECT_NAME}/beaglebone.cpp
# )

## Declare a cpp executable
# add_executable(beaglebone_node src/beaglebone_node.cpp)
add_executable(Protocol_Controller_CAN src/Protocol_Controller_CAN.cpp src/base_node.cpp src/CAN_Driver.cpp)
add_executable(Control_Flow src/Control_Flow.cpp)
add_executable(Module_Info src/Module_Info.cpp src/base_node.cpp)
add_executable(Sanity_Check src/Sanity_Check.cpp src/base_node.cpp)
add_executable(Inverse_Kinematics src/Inverse_Kinematics.cpp src/base_node.cpp)
add_executable(Error_Handler src/Error_Handler.cpp src/base_node.cpp)
add_executable(State_Estimation src/State_Estimation.cpp src/base_node.cpp)
add_executable(State_Estimation_OLD src/State_Estimation_OLD.cpp src/base_node.cpp)
add_executable(Motion_Controller src/Motion_Controller.cpp src/base_node.cpp)
add_executable(Motion_Planner src/Motion_Planner.cpp src/base_node.cpp)
add_executable(LRS_Converter src/LRS_Converter.cpp src/base_node.cpp)
add_executable(Obstruction src/Obstruction.cpp src/base_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(Protocol_Controller_CAN beaglebone_generate_messages_cpp)
add_dependencies(Control_Flow beaglebone_generate_messages_cpp)
add_dependencies(Module_Info beaglebone_generate_messages_cpp)
add_dependencies(Sanity_Check beaglebone_generate_messages_cpp)
add_dependencies(Inverse_Kinematics beaglebone_generate_messages_cpp)
add_dependencies(Error_Handler beaglebone_generate_messages_cpp)
add_dependencies(State_Estimation beaglebone_generate_messages_cpp)
add_dependencies(State_Estimation_OLD beaglebone_generate_messages_cpp)
add_dependencies(Motion_Controller beaglebone_generate_messages_cpp)
add_dependencies(Motion_Planner beaglebone_generate_messages_cpp)
add_dependencies(LRS_Converter beaglebone_generate_messages_cpp)
add_dependencies(Obstruction beaglebone_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(Protocol_Controller_CAN ${catkin_LIBRARIES})

target_link_libraries(Control_Flow ${catkin_LIBRARIES})
target_link_libraries(Module_Info ${catkin_LIBRARIES})
target_link_libraries(Sanity_Check ${catkin_LIBRARIES})
target_link_libraries(Inverse_Kinematics ${catkin_LIBRARIES})
target_link_libraries(Error_Handler ${catkin_LIBRARIES})
target_link_libraries(State_Estimation ${catkin_LIBRARIES})
target_link_libraries(State_Estimation_OLD ${catkin_LIBRARIES})
target_link_libraries(Motion_Controller ${catkin_LIBRARIES})
target_link_libraries(Motion_Planner ${catkin_LIBRARIES})
target_link_libraries(LRS_Converter ${catkin_LIBRARIES})
target_link_libraries(Obstruction ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beaglebone beaglebone_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

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
