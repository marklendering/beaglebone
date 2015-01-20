#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "std_msgs/Bool.h"

#include <stdio.h>
#include <string.h>

//include base class
#include "base_node.h"

using namespace std;

class sanity_check : public base_node {

private:
	ros::NodeHandle _n;
	ros::Subscriber speed_left_right_sub,  obstruction_sub;
	ros::Publisher SpeedLR_pub;
	beaglebone::SpeedLR SpeedLR_to_CAN_msg, SpeedLR_STOP_to_CAN_msg;
	float maxVelocity;
	float SpeedLeftStore, SpeedRightStore, previous_SpeedLeftStore, previous_SpeedRightStore;
	bool obstructed, previous_obstructed, speed_dirty;
	int counterstrike;

public:
	sanity_check(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n){
		maxVelocity = 15.0;
		obstructed = true;
		previous_obstructed = true;
		speed_dirty = false;
		counterstrike = 0;

		SpeedLR_STOP_to_CAN_msg.speedLeft = 0;
		SpeedLR_STOP_to_CAN_msg.speedRight = 0;
		SpeedLR_to_CAN_msg.speedLeft = 0;
		SpeedLR_to_CAN_msg.speedRight = 0;

		SpeedLR_pub = _n.advertise<beaglebone::SpeedLR>("SpeedLeftRightToCAN", 100);
		speed_left_right_sub = _n.subscribe("SpeedLeftRight", 100, &sanity_check::speedLeftRightCallback, this);
		obstruction_sub = _n.subscribe("Obstructed", 10, &sanity_check::ObstructionCallback, this);
	}

	~sanity_check(){
	}

	void speedLeftRightCallback(const beaglebone::SpeedLR::ConstPtr& msg){
		speed_dirty = true;
		char temp[25];

		if( fabs(msg->speedLeft) < 0.1){
			SpeedLeftStore = 0.0;												//speed too low
		}
		else if ( fabs(msg->speedLeft) > maxVelocity){
			Error_srv.request.nodeName = node_name;								//speed too high
			Error_srv.request.errorMessage = "Speed higher than Vmax";

			printf(temp, "speedLeft %5.5f", msg->speedLeft);
			Error_srv.request.aditionalInfo = temp;
			Error_client.call(Error_srv);

			SpeedLeftStore = maxVelocity;
		}
		else {																	//speed OK
			SpeedLeftStore = msg->speedLeft;
		}

		if( fabs(msg->speedRight) < 0.1){
			SpeedRightStore = 0.0;												//speed too low
		}
		else if( fabs(msg->speedRight) > maxVelocity){

			Error_srv.request.nodeName = node_name;								//speed too high
			Error_srv.request.errorMessage = "Speed higher than Vmax";

			printf(temp, "speedRight %5.5f", msg->speedRight);
			Error_srv.request.aditionalInfo = temp;
			Error_client.call(Error_srv);

			SpeedRightStore = maxVelocity;
		}
		else {																	//speed OK
			SpeedRightStore = msg->speedRight;
		}
	}

	void ObstructionCallback(const std_msgs::Bool::ConstPtr& Obstruction_msg){
		obstructed = Obstruction_msg->data;										//check if goal is reached or obstructed
	}

	void errorHook(){
	}

	void updateHook(){
		if(previous_obstructed != obstructed ||	speed_dirty == true ){
			if(obstructed == true){												//If obstructed, publish stop message
				SpeedLR_pub.publish(SpeedLR_STOP_to_CAN_msg);
				printf("Publish stop %i\n\r", counterstrike);
			}
			else{																//If not, publish movement orders
				SpeedLR_to_CAN_msg.speedLeft = SpeedLeftStore;
				SpeedLR_to_CAN_msg.speedRight = SpeedRightStore;
				SpeedLR_pub.publish(SpeedLR_to_CAN_msg);
				printf("Publish go %i\n\r", counterstrike);
			}
			previous_obstructed = obstructed;
			speed_dirty = false;
			counterstrike++;
		}
	}

	void configureHook(){
		SpeedLR_pub = _n.advertise<beaglebone::SpeedLR>("SpeedLeftRightToCAN", 100);
		speed_left_right_sub = _n.subscribe("SpeedLeftRight", 100, &sanity_check::speedLeftRightCallback, this);
		obstruction_sub = _n.subscribe("Obstructed", 10, &sanity_check::ObstructionCallback, this);

		//some init value of the sanity check...
	}

	void resetHook(){
	SpeedLR_pub.shutdown();
	speed_left_right_sub.shutdown();
	}

	void startHook(){
	}

	void stopHook(){
	}

	float fabs(float x){
	return (x<0) ? -x : x;
	}

};

int main(int argc, char ** argv){
	ros::init(argc, argv, "Sanity_Check");
	ros::NodeHandle n;

	sanity_check sanity(n, "sanity_check", 100);

	sanity.loop();

	return 0;
}
