demconRobot
===========



-------------------
quick install notes
-------------------

This section describes how to install this software package on a new machine. If the software 
was already present, you can skip this section.

- Install ubuntu 14.04-LTS
_http://ubuntu.com/_

- Install ros indigo
_http://www.ros.org/install/_

- Setup a ROS catkin workspace in your directory
_http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment_

- Setup .bashrc to automatically load your workspace:
	- Source the workspace
	___source ~/catkin_ws/devel/setup.bash___

- Install dependencies beaglebone
	-apt-get install roscpp rospy rostest tf tf2

- Install dependencies PC
	-apt-get install ros-indigo-global-planner ros-indigo-gmapping ros-indigo-dwa-local-planner ros-indigo-move-base ros-indigo-controller-manager ros-indigo-base-local-planner ros-indigo-costmap-2d

- Clone beaglebone directory in the Catkin_ws/src folder

- Build the project
>_:~/catkin_ws $ catkin_make_
