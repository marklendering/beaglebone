

#include <player_node.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


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

		wheelRadius = 0.0;
		private_nh.getParam("demcon_config/WheelRadius", wheelRadius);
		ROS_INFO("I get wheel radius: [%f]", wheelRadius);

		wheelDis = 0.0;
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

		float left = (2*g_vel - t_vel*wheelDis) / (2*wheelRadius);
		float right = (t_vel*wheelDis + 2*g_vel) / (2*wheelRadius);
		//publish speeds to motors

		beaglebone::WheelVelocities msg;
		msg.left = left;
		msg.right = right;
		WheelVelocities_pub.publish(msg);
		//convert speed to speed per wheel and publish data
	}

	void PlayerNode::deltaDistanceReceived(const beaglebone::WheelDistances::ConstPtr& dist)
	{
		deltaDistR += dist -> right;
		deltaDistL += dist -> left;
	}

	void PlayerNode::publishOdometry()
	{

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

		m_theta += deltaAngle;
		m_x += delta_x;
		m_y += delta_y;

		/*
		std::time_t t = std::time(0);
		double vx = 0.0;
		double vy = 0.0;
		double vth = 0.0;
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;

		//todo calculate delta values for x, y and theta

		double delta_x = (vx * cos(m_theta) - vy* sin(m_theta))*dt;
		double delta_y = (vx * sin(m_theta) + vy * cos(m_theta))*dt;
		double deltaAngle = vth * dt;

		m_y += delta_x;
		m_x += delta_y;
		m_theta += deltaAngle;
		*/


		geometry_msgs::Quaternion odom_quat;
		odom_quat=tf::createQuaternionMsgFromYaw(m_theta);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id="odom";
		odom_trans.child_frame_id="base_footprint";

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

	}
}
