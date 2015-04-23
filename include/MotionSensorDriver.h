#ifndef MOTIONSENSORDRIVER_H
#define MOTIONSENSORDRIVER_H


#include <ros/ros.h>
#include <CanDriver.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include "beaglebone/IMU.h"
#include "beaglebone/LRS.h"
#include "beaglebone/WheelVelocities.h"
#include "beaglebone/WheelDistances.h"
#include <boost/bind.hpp>

#define PI 3.14159265359

#define MCLEFT			0x50
#define MCRIGHT			0x51
#define BEAGLEBONE		0x52
#define ALLMBEDS		0xFF

#define LRSI			0x55
#define LRSR			0x56
#define ANS_DISTR		0x5A
#define ANS_DISTL		0x5B

#define COMMAND_WRITE	'!'
#define COMMAND_READ	'?'
#define GET_LRS			0x57
#define COMMAND_SET_VELOCITY        'v'
#define START		1
#define CONFIGURE	0
#define STOP		2
#define RESET 		3
#define AKM		0x63
#define ANS_AKM		0x62
#define IMUACC		0x61
#define ANS_ACC		0x60
#define IMUGYR		0x59
#define ANS_GYR		0x58

namespace DemconRobot
{
	class MotionSensorDriver
	{
		public:
			ros::NodeHandle node_;
			ros::NodeHandle priv_nh;
			ros::Subscriber WheelVelocities_sub;

			ros::Publisher LRS_RAW_pub;
			ros::Publisher Laser_pub;
			ros::Publisher Imu_pub;
			ros::Publisher IMU_RAW_pub;
			ros::Publisher WheelVelocities_pub;
			ros::Publisher WheelDistances_pub;

			beaglebone::LRS LRS_msg;
			std::string frame_id;
			std::string imu_id;

			MotionSensorDriver();
			~MotionSensorDriver();
			int start();
			int stop();
			void doUpdate();

			bool initRobot();
			void setRobotState(int state);
			void setRobotMode(int mode);
			float getLeftMotorDistance();
			float getRightMotorDistance();
		private:
			CanDriver* canDriver;
			sensor_msgs::LaserScan scan;
			sensor_msgs::Imu imu;
			void getCanData();
			void getLRSData(char readBuffer[8], int length, int canId);
			void requestCANData(int canId);
			void requestCANData(char command, int canId);
			bool getSpeed(char readBuffer[8], int length, int canId);
			void setSpeed(const beaglebone::WheelVelocities::ConstPtr& WheelVelocity);
			void getIMU(char readBuffer[8], int length, int canId);
			void convertLRStoLaserScan(beaglebone::LRS LRS_RAW);
			void getMotorDistance(char readBuffer[8], int length, int canId);
			uint8_t reset_counter;
			bool LRSR_set, LRSI_set, DISTR_set, DISTL_set, speedR_set, speedL_set;
			bool start_flag, finish_flag;
			bool acc_set, gyr_set;
			float Adata[3];
			float Gdata[3];
			int itter, loop_count;
			float Range_buffer[360];
			float Intens_buffer[360];
			float deltaDistanceRight;
			float deltaDistanceLeft;
			float speedRight;
			float speedLeft;
			float create_float_from_bytes(char * buf, int offset); 
	};
	
	union{
		float Float;
		int Int;
	} converter;
}




#endif
