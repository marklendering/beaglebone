
/*
 *
 * X.cpp
 *
 * Created on: dd/mm/yyyy
 * Author: name, companyname
 */

/*
 * All the includes that are being used
 */


#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/CurrentState.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265

//include base class
#include "base_node.h"

using namespace std;

class State_Estimation_OLD : public base_node {

private:
	ros::NodeHandle _n;
	ros::Publisher set_current_state_pub, Path_pub;
	ros::Subscriber get_speed_CAN_sub;
	beaglebone::CurrentState pub_msg;
	std_msgs::Header Header_msg;
	geometry_msgs::Quaternion Quaternion_msg;
	geometry_msgs::Point Point_msg;
	geometry_msgs::Pose Pose_msg;
	geometry_msgs::PoseStamped PoseStamped_msg;
	nav_msgs::Path Path_msg;
	float speedLeft, speedRight, positionX, positionY, positionTheta, speed, angular, wheel_distance, wheel_diameter, time_step, distance;
	int itter;

public:
	State_Estimation_OLD(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
		set_current_state_pub = _n.advertise<beaglebone::CurrentState>("CurrentState_OLD",10);
		Path_pub = _n.advertise<nav_msgs::Path>("Path_oldschool", 10);
		get_speed_CAN_sub = _n.subscribe("SpeedLeftRightFromCAN", 10, &State_Estimation_OLD::getSpeedCallback, this);

		speedLeft = 0.0;
		speedRight = 0.0;
		positionX = 0.0;
		positionY = 0.0;
		positionTheta = 0.0;
		speed = 0.0;
		angular = 0.0;
		wheel_distance = 0.71;
		wheel_diameter = 0.175;
		time_step = (1.0 / (float)period);

		itter = 0;
		Header_msg.seq = itter++;
		Header_msg.frame_id = "/laser";
	}

	~State_Estimation_OLD(){
	}

	void errorHook(){
	}

	void updateHook(){
	}

	void configureHook(){

		//some init value of the state estimation...
		// like wheel_distance and wheel_diameter
		double temp;
		node.getParam("wheelDistance", temp);
		//    wheel_distance = (float) temp;
		node.getParam("wheelDiameter", temp);
		//    wheel_diameter = (float) temp;

	}

	void resetHook(){
		positionX = 0.0;
		positionY = 0.0;
		positionTheta = 0.0;
		speed = 0.0;
		angular = 0.0;
		speedLeft = 0.0;
		speedRight = 0.0;
	}

	void startHook(){
	}

	void stopHook(){
	}

	void getSpeedCallback(const beaglebone::SpeedLR::ConstPtr& msg){
		time_step = (1.0 / (float)period);
																	//from rad/s to m/s
		speedLeft = msg->speedLeft * (wheel_diameter / 2);        	//wl * r = Vl
		speedRight = msg->speedRight * (wheel_diameter / 2);     	//wr * r = Vr

																	//speed & angular calculation
		speed = (speedRight + speedLeft) / 2;                     	//V = (Vr + Vl) / 2
		angular = (speedRight - speedLeft) / wheel_distance;      	//w = (Vr - Vl) / L

		if((angular < 0.001) && (angular > -0.001)){				//position calculation without a radius calculation (speed and angular aren't zero)
			//if(angular <= 0.0){
			//calculate the distance and position
			distance = speed * time_step;                           //s = V * t

			//printf("test %f * %f\n", distance, cos(positionTheta));
			positionX = positionX - (distance * sin(positionTheta));             //x = -(s * sin(theta))
			positionY = positionY + (distance * cos(positionTheta));             //y = s * cos(theta)
		}
		else {
			//calculate angle
			positionTheta += angular * time_step;                   //Theta = w * t

			//check if theta is outside the range -PI to PI
			if(positionTheta > (PI)){
				positionTheta -= (2 * PI);
			}
			if(positionTheta < -(PI)){
				positionTheta += (2 * PI);
			}

			//implement right here the radius calculation
			if((speed > 0.001) || (speed < -0.001)){
				float Radius = speed / angular;                         //R = V / w
				float angle = angular * time_step;                      //alpha = w * t
				float tempX = Radius - (cos(angle) * Radius);           //x = R - (cos(alpha) * R)
				float tempY = sin(angle) * Radius;                      //y = sin(alpha) * R
				float tempDistance = sqrt(pow(tempX, 2.0) + pow(tempY, 2.0)); //pythagoras

				tempDistance = (tempY < 0) ? -tempDistance : tempDistance;	//determine forward or backward

				//	printf( "x: %f y: %f d: %f\n", tempX, tempY, tempDistance );

				positionX -= tempDistance * sin(positionTheta);             //x = -(s * sin(theta))
				positionY += tempDistance * cos(positionTheta);             //y = s * cos(theta)
			}
		}

		//dummy function publish
		pub_msg.x = positionX;
		pub_msg.y = positionY;
		pub_msg.theta = positionTheta;
		pub_msg.speed = speed;
		pub_msg.angular = angular;

		//publish message for old system
		set_current_state_pub.publish(pub_msg);

		//Building the Path message from bottom up
		Quaternion_msg.x = 0;
		Quaternion_msg.y = 0;
		Quaternion_msg.z = angular;
		Quaternion_msg.w = 0;

		Point_msg.x = positionX;
		Point_msg.y = positionY;
		Point_msg.z = 0;

		Pose_msg.position = Point_msg;
		Pose_msg.orientation = Quaternion_msg;

		Header_msg.seq = itter++;					//set sequence number
		Header_msg.stamp = ros::Time::now();		//set time in the header

		PoseStamped_msg.pose = Pose_msg;
		PoseStamped_msg.header = Header_msg;

		Path_msg.header = Header_msg;
		Path_msg.poses[0] = PoseStamped_msg;

		Path_pub.publish(Path_msg);
	}

};

int main(int argc, char ** argv){
	ros::init(argc, argv, "State_Estimation_OLD");
	ros::NodeHandle n;

	State_Estimation_OLD state(n, "State_Estimation_OLD", 30);

	state.loop();

	return 0;
}


