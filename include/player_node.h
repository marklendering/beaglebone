#ifndef PLAYERNODE_H
#define PLAYERNODE_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <beaglebone/WheelVelocities.h>
#include <beaglebone/WheelDistances.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

namespace DemconRobot
{
	class PlayerNode
	{
		public:
			//tf::TranformBroadcaster m_odom_broadcaster;
			//tf::Transform transform;

			ros::NodeHandle node_;
			ros::Subscriber cmd_vel_sub_;
			ros::Subscriber WheelDistances_sub;
			ros::Publisher WheelVelocities_pub;

			PlayerNode();
			~PlayerNode();

			int start();
			int stop();
			void doUpdate();

			void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
			void deltaDistanceReceived(const beaglebone::WheelDistances::ConstPtr& dist);



		private:
			//ros
			ros::Publisher odom_pub;
			ros::Time last_time;

			//private methods
			void publishOdometry();
			void addRangeToCloud(const sensor_msgs::Range& range, sensor_msgs::PointCloud& pointcloud);
			void calculateMovementDelta();

			//private variables
			double wheelRadius;
			double wheelDis;
			float deltaDistR;
			float deltaDistL;
			float m_y;
			float  m_x;
			float  m_theta;
			std::string robotType;
			std::string robotId;
			double minSpeed;
			double maxSpeed;
	};
}


#endif
