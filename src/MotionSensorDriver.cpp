
#include <MotionSensorDriver.h>

using namespace std;

namespace DemconRobot
{
	MotionSensorDriver::MotionSensorDriver()
	{
		canDriver = new CanDriver();
		priv_nh.param("frame_id", frame_id, std::string("/laser_scanner"));
		priv_nh.param("imu_id", imu_id, std::string("/base_imu"));
		scan.angle_min=-PI;
		imu.header.frame_id = imu_id;
		scan.angle_max=PI;
		scan.range_min=0.30;
		scan.range_max=5.0;
		scan.angle_increment=PI/180;
		scan.header.frame_id=frame_id;
		scan.time_increment=0;
		scan.ranges.resize(360);
		scan.intensities.resize(360);

		LRSR_set = false;
		speedR_set = false;
		speedL_set = false;
		LRSI_set = false;
		DISTR_set = false;
		DISTL_set = false;
		start_flag = false;
		finish_flag = false;
		reset_counter = 0;
		deltaDistanceRight = 0;
		deltaDistanceLeft = 0;
		speedRight = 0.0;
		speedLeft = 0.0;
		//IMU_RAW_pub = node_.advertise<SOMETHING>("IMU_RAW", 100);
		Laser_pub = node_.advertise<sensor_msgs::LaserScan>("/laser/scan", 1000);
		Imu_pub = node_.advertise<sensor_msgs::Imu>("/imu_data", 1); //1000
		//LRS_RAW_pub = node_.advertise<beaglebone::LRS>("LRS_RAW", 100);
		//replace LRS_RAW_pub with LaserScan 
		WheelVelocities_pub = node_.advertise<beaglebone::WheelVelocities>("Wheel_Current_Velocities", 100);
		WheelDistances_pub = node_.advertise<beaglebone::WheelDistances>("Wheel_Delta_Distance", 100);

	}

	MotionSensorDriver::~MotionSensorDriver()
	{
		delete canDriver;
	}

	int MotionSensorDriver::start()
	{
		if(!canDriver->open_Bus((char*)"can1"))
		{
			return 1;
		}
		WheelVelocities_sub = node_.subscribe<beaglebone::WheelVelocities>("Wheel_Set_Velocity", 100, boost::bind(&MotionSensorDriver::setSpeed, this, _1));
		initRobot();
		return 0;
	}

	int MotionSensorDriver::stop()
	{
		return 0;
	}

	void MotionSensorDriver::doUpdate()
	{
		requestCANData(GET_LRS);
		getCanData();
		getCanData();
		requestCANData('D', ALLMBEDS);
		getCanData();
		getCanData();
		//requestCANData('V', ALLMBEDS);
		//getCanData();
		//getCanData();
		requestCANData(IMUACC);
		getCanData();
		requestCANData(IMUGYR);
		getCanData();


		//request candata for distance traveled
		//getdata for mbed one
		//getdata for mbed two
		//

		//get LRS data every tick
		//every 10 ticks, update rest of data.

		//get LRS data
		//get Speed data
		//get IMU data

	}

	void MotionSensorDriver::requestCANData(int canId)
	{
		//request data from devices on canbus
		char buf[8];
		canDriver->write_Bus(buf, 0, canId);
	}

	void MotionSensorDriver::requestCANData(char command, int canId)
	{
		char buf[8];
		//set buffer to command settings
		buf[0] = '?';
		buf[1] = command;
		canDriver->write_Bus(buf, 2, canId);
	}

	void MotionSensorDriver::getLRSData(char readBuffer[8], int length, int canId)
	{
		if(!start_flag && readBuffer[0]==0) start_flag = true;
		if(length == 8 && start_flag)
		{
			if(canId == LRSR)
			{
				scan.ranges.push_back( (float(readBuffer[2]<<8|readBuffer[3]))/1000);
				scan.ranges.push_back( (float(readBuffer[4]<<8|readBuffer[5]))/1000);
				scan.ranges.push_back( (float(readBuffer[6]<<8|readBuffer[7]))/1000); 
				LRSR_set = true;
			}
			else if(canId ==LRSI)
			{
				scan.intensities.push_back((readBuffer[2]<<8)|readBuffer[3]);
				scan.intensities.push_back((readBuffer[4]<<8)|readBuffer[5]);
				scan.intensities.push_back((readBuffer[6]<<8)|readBuffer[7]);
				LRSI_set=true;
			}
		}
		if(LRSI_set && LRSR_set){
			reset_counter++;
			if(reset_counter == 120){
				//scan.header.stamp = ros::Time(0);		//set time in the header
				scan.header.stamp = ros::Time::now();		//set time in the header
				Laser_pub.publish(scan);
				scan.ranges.clear();
				scan.intensities.clear();
				reset_counter = 0;
				start_flag = false;
				//printf("sent package! \n \r");
			}
			LRSI_set = false;
			LRSR_set = false;
		}
	}

	void MotionSensorDriver::getCanData()
	{
		//read data from canbus and handle accordingly
		char readBuffer[8];
		int length = 0;
		int canId= 0;
		if(canDriver->read_Bus(readBuffer, length, canId))
		{
			switch(canId)
			{
				case LRSI: case LRSR :
					getLRSData(readBuffer, length, canId);
					break;
				case ANS_DISTR: case ANS_DISTL:
					getMotorDistance(readBuffer, length, canId);
					break;
				case BEAGLEBONE:
					getSpeed(readBuffer, length, canId);
					break;
				case ANS_GYR: case ANS_ACC: case ANS_AKM:
					getIMU(readBuffer, length, canId);
					break;
				default:
					printf("incorrect canId: %d \r \n", canId);
					break;
			}
		}
	}

	void MotionSensorDriver::getIMU(char readBuffer[8], int length, int canId)
	{
		if(canId == ANS_ACC)
		{
			acc_set = true;
			Adata[0] = create_float_from_bytes(readBuffer, 0);
			Adata[1] = create_float_from_bytes(readBuffer, 2);
			Adata[2] = create_float_from_bytes(readBuffer, 4);
		}

		else if(canId == ANS_GYR)
		{
			gyr_set = true;
			Gdata[0] = create_float_from_bytes(readBuffer, 0);
			Gdata[1] = create_float_from_bytes(readBuffer, 2);
			Gdata[2] = create_float_from_bytes(readBuffer, 4);
		}

		if(acc_set && gyr_set)
		{
			imu.header.stamp = ros::Time::now() - ros::Duration(0.6);

			geometry_msgs::Quaternion orientation;// = tf::createQuaternionMsgFromYaw(0.0);
			gyr_set = false;
			acc_set = false;

			//imu.header.stamp = ros::Time::now();
			//imu.orientation = tf::createQuaternionMsgFromYaw(0.0);
			float roll = Gdata[0] * (PI/(131 * 180));
			float pitch = Gdata[1] * (PI/(131 * 180));
			float yaw = Gdata[2] * (PI/(131*180));
			imu.angular_velocity.x = roll;
			imu.angular_velocity.y = pitch;
			imu.angular_velocity.z = yaw;
			//imu.angular_velocity_covariance[0] = 0.001;
			//imu.angular_velocity_covariance[4] = 0.001;
			//imu.angular_velocity_covariance[8] = 0.001;
			//tf::Quaternion orientation_quat = tf.transformations.quaternion_from_euler(0,0,yaw);
			//imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
			imu.orientation = tf::createQuaternionMsgFromYaw(yaw);
			//imu.orientation.x = orientation_quat[0];
			//imu.orientation.y = orientation_quat[1];
			//imu.orientation.z = orientation_quat[2];
			//imu.orientation.w = orientation_quat[3];
			//imu.orientation_covariance[0] = 0;
			//imu.orientation_covariance[4] = 0;
			//imu.orientation_covariance[8] = 0.001;

			//imu.orientation = orientation;

			imu.linear_acceleration.x = Adata[0] * (9.81/16834);
			imu.linear_acceleration.y = Adata[1] * (9.81/16834);
			imu.linear_acceleration.z = 0;//Adata[2] * (9.81/16834);
			//imu.linear_acceleration_covariance[0] = 0.001;
			//imu.linear_acceleration_covariance[4] = 0.001;
			//imu.linear_acceleration_covariance[8] = 0.001;


			//set orientation covariance
			//set angular velocity covariance
			//set linear velocity covariance
			Imu_pub.publish(imu);

		}
		//get gyro and accelerometer data
		//convert data to correct format
		//create imu message and publish it.


	}

	void MotionSensorDriver::getMotorDistance(char readBuffer[8], int length, int canId)
	{
		if(length >= 5)
		{
			converter.Int = ((readBuffer[1]<<0) + (readBuffer[2]<<8) + (readBuffer[3]<<16) + (readBuffer[4]<<24));
			if(canId == ANS_DISTR)
			{
				DISTR_set = true;
				deltaDistanceRight = converter.Float;
			}
			else if(canId == ANS_DISTL)
			{
				DISTL_set = true;
				deltaDistanceLeft = converter.Float;
			}

			if(DISTR_set && DISTL_set)
			{
				beaglebone::WheelDistances msg;
				msg.right = deltaDistanceRight;
				msg.left = deltaDistanceLeft;
				//printf("distanceLeft: %f , distanceRight: %f \r \n", deltaDistanceLeft, deltaDistanceRight);
				WheelDistances_pub.publish(msg);
				DISTR_set = false;
				DISTL_set = false;
			}
		}
	}

	bool MotionSensorDriver::getSpeed(char readBuffer[8], int length, int canId)
	{
		char int_buffer[4];
		if(length >=5)
		{
			int_buffer[0] = readBuffer[1];
			int_buffer[1] = readBuffer[2];
			int_buffer[2] = readBuffer[3];
			int_buffer[3] = readBuffer[4];
			converter.Int = ((int_buffer[0] << 0) + (int_buffer[1] << 8) + (int_buffer[2] << 16) + (int_buffer[3] << 24));
			if(readBuffer[0] == MCRIGHT)
			{
				speedR_set = true;
				speedRight = converter.Float;
		//set bool and variable
			}
			else if(readBuffer[0] == MCLEFT)
			{
				speedL_set = true;
				speedLeft = converter.Float;
				//set bool and variable
			}
			if(speedR_set && speedL_set)
			{
				speedL_set = false;
				speedR_set = false;
				beaglebone::WheelVelocities msg;
				msg.right = speedRight;
				msg.left = speedLeft;
				//WheelVelocities_pub.publish(msg);
				//printf("speedRight: %f , speedLeft: %f \r \n", speedRight, speedLeft);
			}
			//if both bools set publish speed, or for now, just print speed.
		}
		return false;
	}

	void MotionSensorDriver::setSpeed(const beaglebone::WheelVelocities::ConstPtr& WheelVelocity)
	{
		//TODO: not very efficient
		char buffer[6];
		//left motor
		buffer[0]=COMMAND_WRITE;
		buffer[1]=COMMAND_SET_VELOCITY;

		converter.Float = WheelVelocity -> left;
		buffer[2] = ((converter.Int >> 0) & 0xff);
		buffer[3] = ((converter.Int >> 8) & 0xff);
		buffer[4] = ((converter.Int >> 16) & 0xff);
		buffer[5] = ((converter.Int >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, MCLEFT);

		//right motor
		converter.Float = WheelVelocity -> right;
		buffer[2] = ((converter.Int >> 0) & 0xff);
		buffer[3] = ((converter.Int >> 8) & 0xff);
		buffer[4] = ((converter.Int >> 16) & 0xff);
		buffer[5] = ((converter.Int >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, MCRIGHT);
		//printf("left speed: %f, right speed: %f \n \r", WheelVelocity->left, WheelVelocity->left);
		setRobotState(START);

	}

	//getIMU data -> temp, gyro, accelerometer, compass data

	//add method to convert raw LRS data to laserscan data, then publish this data

	bool MotionSensorDriver::initRobot()
	{
		setRobotState(CONFIGURE);
		setRobotMode(1);
	}

	void MotionSensorDriver::setRobotMode(int mode)
	{
		char buffer[6];
		buffer[0] = '!';
		buffer[1] = 'm';
		buffer[2] = ((mode >> 0) & 0xff);
		buffer[3] = ((mode >> 8) & 0xff);
		buffer[4] = ((mode >> 16) & 0xff);
		buffer[5] = ((mode >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, ALLMBEDS); 
	}

	void MotionSensorDriver::setRobotState(int state)
	{
		char buffer[2];
		buffer[0] = '!';
		switch(state)
		{
			case CONFIGURE:
				buffer[1] = 'c';
				break;
			case START:
				buffer[1] = 's';
				break;

			case STOP:
				buffer[1] = 't';
				break;

			case RESET:
				buffer[1] = 'r';
				break;

			default:
				buffer[1] = 't';
				break;
		}
		canDriver->write_Bus(buffer, 2, ALLMBEDS);

	}

	float MotionSensorDriver::create_float_from_bytes(char * buf, int offset)
	{
		unsigned int b0, b1;
		b0 = buf[offset];
		b1 = buf[offset+1];
		unsigned short total = (b0 <<8) | b1;

		if((total&0x8000)==0x8000)
		{
			total = ((total xor 0xFFFF)+1);
			total *= -1;
		}

		signed short stotal = total;
		return (float)stotal;
	}

}
