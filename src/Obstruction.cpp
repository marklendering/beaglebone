
/*
 *
 * obstruction.cpp
 *
 * Created on: dd/mm/yyyy
 * Author: name, companyname
 */

/*
 * All the includes that are being used
 */

#include "ros/ros.h"
#include "base_node.h"
#include "beaglebone/LaserScanB.h"
#include "std_msgs/Bool.h"


using namespace std;

class obstruction : public base_node {

private:
	ros::NodeHandle _n;


	//ROS Subscriber
	ros::Subscriber LaserScan_sub;

	//ROS Publisher
	ros::Publisher Obstructed_pub;

	//ROS ServiceServer
	//ros::ServiceServer test_;

	//ROS ServiceClient
	//ros::ServiceClient test_;

	/*
	* Used attributes
	*/
	int test;

public:
	obstruction(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n){

	  //define subscribtions
	  LaserScan_sub = _n.subscribe("LRS_scan",100, &obstruction::stopScanCallback, this);

	  //define publishers
	  Obstructed_pub = _n.advertise<std_msgs::Bool>("Obstructed", 100);

	}

// Destructor
	~obstruction(){}

// State methods that are executed at the loop rate when the node is in a specific state
	void init()
	{
	}

	void idle()
	{
	}

	void run()
	{
	}

	void error()
	{
	}

// state transition methods that are executed once when changing to a state
	void errorHook(){
	}

	void updateHook(){

	}

	void configureHook(){

	}

	void resetHook(){

	}

	void startHook(){

	}

	void stopHook(){

	}

//examples functions for service, callback and other functions
  /*
  * describe callback
  */
	void stopScanCallback(const beaglebone::LaserScanB::ConstPtr& LaserMessage){

		int scan_range = 90;
		bool obstruction = false;
		vector<float> Range_v = LaserMessage->ranges;
		std_msgs::Bool Obstruction_msg;

		for(int i = 0 ; i < scan_range/2; i++){		//scanning for obstructions from 0 to...
			if(Range_v[i] <=0.40 && Range_v[i] >=0.15 ){
			  obstruction = true;
			}
		}

		for(int i = 360-scan_range/2; i < 360; i++){		//scanning for obstructions from 0 to...
			if(Range_v[i] <=0.40 && Range_v[i] >=0.15 ){
				obstruction = true;
			}
		}

		Obstruction_msg.data = obstruction;
		Obstructed_pub.publish(Obstruction_msg);

	}
};

int main(int argc, char ** argv){
	ros::init(argc, argv, "obstruction");
	ros::NodeHandle n;

	obstruction obstruction(n, "obstruction", 30);

	obstruction.loop();

	return 0;
}
