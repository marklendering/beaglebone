





//#include <assert.h>
//#include <boost/thread.hpp>
//#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <player_node.h>
//#include <MotionSensorDriver.h>

#include <math.h>


using namespace std;
using namespace DemconRobot;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Driver");

	PlayerNode demconRobotPlayer;
	//MotionSensorDriver driver;
	ros::NodeHandle n;

	if(demconRobotPlayer.start()!= 0){
		exit(-1);
	}


	ros::Rate loop_rate(500); //10hz


	while(n.ok()){
		demconRobotPlayer.doUpdate();
		ros::spinOnce();
		loop_rate.sleep();
	}

	demconRobotPlayer.stop();

	return(0);


}
