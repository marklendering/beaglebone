#include <ros/ros.h>
//#include <move_base_msgs/MovebaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <QApplication>
#include <QWidget>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char* argv[]){
	//ros::init(argc, argv, "simple_navigation_goals");
	QApplication app(argc, argv);
	QWidget window;
	
	window.resize(250,150);
	window.setWindowTitle("simple example");
	window.show();
	/*
	MoveBaseClient ac("move_base, true);
	
	while(!ac.waitForServer(ros::Duration(5.0)){
		ROS_INFO("waiting for the move_base action server to come up);
	}
	
	move_base_msgs::MoveBaseGoal goal;
	
	goal.target_pose.header.frame_id="base_footprint";
	goal.target_pose.header.stamp=ros::Time::now();
	
	goal.target_pose.pose.position.x = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;
	
	ROS_INFO("sending goal");
	ac.sendGoal(goal);
	
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("moved 1 meter forward");
	else
		ROS_INFO("the base failed to move to its directed position");
	*/
	
	return app.exec();
}