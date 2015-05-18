

#include <player_node.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#define EKF

using namespace std;

namespace DemconRobot
{
	PlayerNode::PlayerNode()
	{
		ros::NodeHandle private_nh("");
		robotId = "demconRobot1";
		private_nh.getParam("demcon_config/RobotId", robotId);
		ROS_INFO("I get ROBOT_ID: [%s] ", robotId.c_str());

		robotType = "Dcon";
		private_nh.getParam("demcon_config/RobotType", robotType);
		ROS_INFO("I get ROBOT_TYPE: [%s]", robotType.c_str());

		wheelRadius = 0.085;
		private_nh.getParam("demcon_config/WheelRadius", wheelRadius);
		ROS_INFO("I get wheel radius: [%f]", wheelRadius);

		wheelDis = 0.480;
		private_nh.getParam("demcon_config/WheelDistance", wheelDis);
		ROS_INFO("I get wheel distance: [%f]", wheelDis);

		minSpeed = 0;
		private_nh.getParam("demcon_config/MinSpeed", minSpeed);
		ROS_INFO("I get minimum speed: [%f]", minSpeed);

		maxSpeed = 0;
		private_nh.getParam("demcon_config/MaxSpeed", maxSpeed);
		ROS_INFO("I get maximum speed [%f] ", maxSpeed);

		m_y = 0;
		m_x = 0;
		m_theta = 0;
		deltaDistR = 0;
		deltaDistL = 0;
		WheelVelocities_pub = node_.advertise<beaglebone::WheelVelocities>("Wheel_Set_Velocity", 100);
		odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 10);

		last_time=ros::Time::now();
	}

	PlayerNode::~PlayerNode()
	{
	}

	int PlayerNode::start()
	{
		//subscribe to cmd_vel from move_base stack
		cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&PlayerNode::cmdVelReceived, this, _1));
		WheelDistances_sub = node_.subscribe<beaglebone::WheelDistances>("/Wheel_Delta_Distance", 10, boost::bind(&PlayerNode::deltaDistanceReceived, this, _1));
		return 0;
	}

	int PlayerNode::stop()
	{
		return 0;
	}

	void PlayerNode::doUpdate()
	{
		//do something
		//tf::Quaternion odom_quat;
		//odom_quat.setRPY(0,0,m_theta);
		//tf::Transform transform;
		//transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
		//transform.setRotation(odom_quat);

		//tf::TransformBroadcaster m_odom_broadcaster;
		//m_odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
		publishOdometry();
	}

	void PlayerNode::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
	{
		//TODO: check this method for functionality and improvements
		double g_vel = cmd_vel -> linear.x;
		double t_vel = cmd_vel -> angular.z;
/*
		float left = 0;
		float right = 0;
		if(t_vel < 0.0015 && t_vel > -0.0015)
		{
			left = g_vel;
			right = g_vel;
		}
		else
		{
			if(t_vel < 0)
			{
				left = -t_vel;
				right = t_vel;
			}
			else
			{
				left = t_vel;
				right = -t_vel;
			}
		}
*/

		float left = (2*g_vel - t_vel*wheelDis) / (2*wheelRadius);
		float right = (t_vel*wheelDis + 2*g_vel) / (2*wheelRadius);
		//publish speeds to motors
		printf("linear: %f angular: %f \n \r",g_vel, t_vel); 
		printf("left: %f right: %f \n \r", left, right);
		//printf("left: %f, right: %f \n \r", msg.left, msg.right);
		beaglebone::WheelVelocities msg;
		msg.left = left;
		msg.right = right;
		WheelVelocities_pub.publish(msg);
		//convert speed to speed per wheel and publish data
	}

	void PlayerNode::deltaDistanceReceived(const beaglebone::WheelDistances::ConstPtr& dist)
	{
		deltaDistR += ((dist -> right)/100);
		deltaDistL += ((dist -> left)/100);
		//if(deltaDistR !=0 || deltaDistL != 0) printf("distr: %f, distl: %f \n \r", deltaDistR, deltaDistL);
	}

	void PlayerNode::publishOdometry()
	{
		//printf("published odometry");

		//getspeed of motorsideA
		//getspeed of motorsideB

		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;
		double avgDistance = (deltaDistL + deltaDistR) /2.0;
		double deltaAngle = atan2((deltaDistR - deltaDistL), wheelDis);
		deltaDistR = 0;
		deltaDistL = 0;
		double delta_x = avgDistance * cos(m_theta);
		double delta_y = avgDistance * sin(m_theta);

		double vx = avgDistance / dt;
		double vy = 0.0;
		double vth = deltaAngle / dt;

		//deltaAngle = 0;
		m_theta += deltaAngle;
		m_x += delta_x;
		m_y += delta_y;

		#ifndef EKF
		/////////NEW ///////
//		tf::Quaternion odom_quat;
//		odom_quat.setRPY(0,0,m_theta);
//
//		tf::Transform transform;
//		transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
//		transform.setRotation(odom_quat);
//
//		tf::TransformBroadcaster broadcaster;
//		broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_footprint"));
		#endif


		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = m_x;
		odom.pose.pose.position.y = m_y;
		//odom.pose.covariance[0] = 0.001;
		//odom.pose.covariance[7] = 0.001;
		//odom.pose.covariance[14] = 1000000;
		//odom.pose.covariance[21] = 1000000;
		//odom.pose.covariance[28] = 1000000;
		//odom.pose.covariance[35] = 1.0;

		geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(m_theta);
		odom.pose.pose.orientation = odom_quat2;

//		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//odom.twist.covariance[0] = 0.001;
		//odom.twist.covariance[7] = 0.001;
		//odom.twist.covariance[14] = 1000000;
		//odom.twist.covariance[21] = 1000000;
		//odom.twist.covariance[28] = 1000000;
		//odom.twist.covariance[35] = 0.01;
		//odom.twist.covariance = odom.pose.covariance;

		odom_pub.publish(odom);

		/*
		///////OLD WORKING///////
		geometry_msgs::Quaternion odom_quat;
		odom_quat=tf::createQuaternionMsgFromYaw(m_theta);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id="odom";
		odom_trans.child_frame_id="base_footprint";

		current_time = ros::Time::now();
		odom_trans.header.stamp=current_time;
		odom_trans.transform.translation.x=m_x;
		odom_trans.transform.translation.y=m_y;
		odom_trans.transform.translation.z=0.0;
		odom_trans.transform.rotation=odom_quat;

		tf::TransformBroadcaster broadcaster;
		broadcaster.sendTransform(odom_trans);

		nav_msgs::Odometry odom;
		odom.header.stamp=current_time;
		odom.header.frame_id="odom";
		odom.child_frame_id="base_footprint";

		odom.pose.pose.position.x=m_x;
		odom.pose.pose.position.y=m_y;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation=odom_quat;

		odom.twist.twist.linear.x=vx;
		odom.twist.twist.linear.y=vy;
		odom.twist.twist.angular.z=vth;

		odom_pub.publish(odom);
		*/
	}
}
