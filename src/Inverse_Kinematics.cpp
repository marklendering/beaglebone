#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/Velocity.h"

//include base class
#include "base_node.h"

using namespace std;

class inverse_kinematics : public base_node {

private:
	ros::NodeHandle _n;
	ros::Publisher set_speed_pub;
	ros::Subscriber velocity_sub;
	beaglebone::SpeedLR pub_msg;
	float speed, angular, wheel_diameter, wheel_distance;

public:
	inverse_kinematics(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
		set_speed_pub = _n.advertise<beaglebone::SpeedLR>("SpeedLeftRight",100);
		velocity_sub = _n.subscribe("Velocity", 100, &inverse_kinematics::velocityCallback, this);

		wheel_diameter = 0.175;                       //in meters
		wheel_distance = 0.4942;                       //in meters
	}

	~inverse_kinematics(){
	}

	//from cartesisch space to joint space calculation
	//Vl = V - (1/2 * w * L), waar w de angular velocity is en L de wielbasis
	//Vl is in m/s daarna wordt er d.m.v. de straal omgerekent naar rad/s
	void velocityCallback(const beaglebone::Velocity::ConstPtr& msg){
		speed = msg->speed;                                              //V = speed velocity

		angular = ((msg->angular * wheel_distance) / 2);                 //(w * L) / 2

		pub_msg.speedLeft = (speed - angular) / (0.5 * wheel_diameter);  //Conversion from m/s to rad/s (joint space)
		pub_msg.speedRight = (speed + angular) / (0.5 * wheel_diameter); //Conversion from m/s to rad/s (joint space)

		set_speed_pub.publish(pub_msg);
	}

	void errorHook(){
	}

	void updateHook(){
	}

	void configureHook(){


		//some init for inverse kinematics like
		//width car
		//wheeldiameter e.d.
		double temp;
		node.getParam("wheelDiameter",temp);
		//    wheel_diameter = (float) temp;
		//    printf("Got new wheel diameter: %f\n", temp);

		node.getParam("wheelDistance",temp);
		//    printf("Got new wheel distance: %f\n", temp);
		//    wheel_distance = (float) temp;

	}

	void resetHook(){
	}

	void startHook(){
	}

	void stopHook(){
	}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "Inverse_Kinematics");
  ros::NodeHandle n;

  inverse_kinematics inverse(n, "inverse_kinematics", 30);

  inverse.loop();

  return 0;
}
