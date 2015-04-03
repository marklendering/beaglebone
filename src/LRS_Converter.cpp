/*
 *
 * LRS_converter.cpp
 *
 * Created on: 27/05/2013
 * Author: A. Zeinstra, DEMCON
 *
 * Goal of this node is to convert messages of the LRS_CAN_MAILBOX topic
 * to the /sensor_msgs/LaserScan
 * Since the Beaglebone doesn't recognise this, its converting to Beaglebone/LaserScanB.
 * This message is simply copied due to time constrains.
 * Due to simplicity of this message, no state machine is implemented.
 *
 * Message is as follows
 * std_msgs/Header header
 *   uint32 seq
 *   time stamp
 *   string frame_id
 * float32 angle_min
 * float32 angle_max
 * float32 angle_increment
 * float32 time_increment
 * float32 scan_time
 * float32 range_min
 * float32 range_max
 * float32[] ranges
 * float32[] intensities
 */

/*
 * All the includes that are being used
 */

#include "base_node.h"
#include "ros/ros.h"

#include "beaglebone/LaserScanB.h"
#include "std_msgs/Header.h"
#include "beaglebone/LRS.h"

#define PI 3.14159265359

using namespace std;

class LRS_Converter : public base_node {

private:
	ros::NodeHandle _n;

	//ROS Subscriber
	ros::Subscriber LRS_RAW_sub;

	//ROS Publisher
	ros::Publisher LaserScan_pub;

	//ROS ServiceServer

	//ROS ServiceClient

	/*
	* Used attributes
	*/
	bool start_flag, finish_flag;
	float Range_buffer[ 360 ], Intens_buffer[ 360 ];
	int measurements, angle_min, angle_max, itter, loop_count;

	beaglebone::LaserScanB LaserScan_msg;

public:
	LRS_Converter(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
		//ROS_ERROR_STREAM("init \n\r");
		//set up topic
		LRS_RAW_sub = _n.subscribe("LRS_RAW", 100, &LRS_Converter::LRS_Callback, this);
		LaserScan_pub = _n.advertise<beaglebone::LaserScanB>("LRS_scan", 10);

		loop_count = 0;

		start_flag = false;
		finish_flag = false;

		angle_min = 0;
		angle_max = 360;

		//set standard LaserScan values
		LaserScan_msg.angle_min = 0;//angle_min*((2*PI)/360);	//start angle of the scan [rad]
		LaserScan_msg.angle_max = 2*PI;//angle_max*((2*PI)/360);	//end angle of the scan [rad]
		LaserScan_msg.angle_increment= (PI/180);			//angular distance between measurements [rad]
		LaserScan_msg.time_increment = 0.2;					//time between measurements [seconds] - if your scanner
															//is moving, this will be used in interpolating position
															//of 3d points
		LaserScan_msg.scan_time = 1/(360*5);						//time between scans [seconds]
		LaserScan_msg.range_min = 0.15;    					//minimum range value [m]
		LaserScan_msg.range_max = 5.0; 						//maximum range value [m]

		itter = 0;
		LaserScan_msg.header.seq = itter;
		LaserScan_msg.header.frame_id = "/laser";
	}
// Destructor
	~LRS_Converter(){}

// State methods that are executed at the loop rate when the node is in a specific state
	void init(){

	}

	void idle(){

	}

	void run(){

	}

	void error(){

	}

// state transition methods that are executed once when changing to a state
	void errorHook(){

	}

	void updateHook(){

	}

	void configureHook(){

	}

	void resetHook(){
		loop_count = 0;

		start_flag = false;
		finish_flag = false;
	}

	void startHook(){

	}

	void stopHook(){

	}

//examples functions for service, callback and other functions
	/*
	* describe callback
	*/
	void LRS_Callback(const beaglebone::LRS::ConstPtr& LRS_RAW_msg)
	{
		int indexR = LRS_RAW_msg->IdxR;
		int indexI = LRS_RAW_msg->IdxI;

		if(indexR != indexI){
			Error_srv.request.nodeName = node_name;
			Error_srv.request.errorMessage = "Index mismatch";
			Error_srv.request.aditionalInfo = "Weird stuff going on in sensorboard";
			Error_client.call(Error_srv);
		}


		if(indexR == 0 && start_flag !=true){			//start index. Ex: 30 = 90deg ; 60 = 180 ; 90 = 270deg
			start_flag = true;
			//ROS_ERROR_STREAM("startF\n\r");
		}

		if(indexR == 120 && start_flag == true){		//end index
			finish_flag = true;
		}

		if(start_flag == true){		//fill the range and intensity buffer array
			Range_buffer[(3*loop_count)+0] 	= LRS_RAW_msg->ran1;
			Range_buffer[(3*loop_count)+1] 	= LRS_RAW_msg->ran2;
			Range_buffer[(3*loop_count)+2] 	= LRS_RAW_msg->ran3;
			Intens_buffer[(3*loop_count)+0] 	= LRS_RAW_msg->intens1;
			Intens_buffer[(3*loop_count)+1] 	= LRS_RAW_msg->intens2;
			Intens_buffer[(3*loop_count)+2] 	= LRS_RAW_msg->intens3;
			loop_count++;
		}

		if(finish_flag == true){		//set variables, publish and reset flags, vectors
			LaserScan_msg.header.seq = itter++;					//set sequence number
			LaserScan_msg.header.stamp = ros::Time::now();		//set time in the header

			int LengthOfBuff = sizeof( Range_buffer ) / sizeof( int );
			for(int meow = 0 ; meow < LengthOfBuff ; meow++){
				LaserScan_msg.ranges.push_back(Range_buffer[meow]);
				LaserScan_msg.intensities.push_back(Range_buffer[meow]);//?? Intens_buffer instead of Range_buffer>>????
			}

			LaserScan_pub.publish(LaserScan_msg);				//publish

			LaserScan_msg.ranges.clear();						//clear vectors
			LaserScan_msg.intensities.clear();

			//ROS_ERROR_STREAM("done sending\n\r");

			loop_count = 0;
			start_flag = false;
			finish_flag = false;
		}
	}

	/*
	* describe service
	*/


	/*
	* describe function
	*/


};

int main(int argc, char ** argv){
	ros::init(argc, argv, "LRS_Converter");
	ros::NodeHandle n;

	LRS_Converter LRS_Converter(n, "LRS_Converter", 100);

	LRS_Converter.loop();

	return 0;
}
