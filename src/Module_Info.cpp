#include "ros/ros.h"

#include "beaglebone/ModuleState.h"
#include "beaglebone/Error.h"
#include "beaglebone/CurrentState.h"
#include "beaglebone/Velocity.h"
#include "beaglebone/SetMode.h"
#include "beaglebone/Position.h"
#include "beaglebone/Empty.h"

//include base class
#include "base_node.h"

using namespace std;

class module_info : public base_node {

private:
	ros::NodeHandle _n;
	ros::Subscriber current_state_sub, error_sub, velocity_sub, set_mode_sub, position_goal_sub;
	ros::Publisher module_state_pub, error_state_pub;
	ros::ServiceServer position_reached_srv;
	ros::ServiceClient finished_clnt;
	beaglebone::ModuleState module_state_msg;
	beaglebone::Error error_msg;
	beaglebone::Empty finished_srv;
	bool verbose;

public:
  module_info(ros::NodeHandle n, string node_name, int period):
	  base_node(node_name, n, period), _n(n)
	{
		//publishers
		module_state_pub = _n.advertise<beaglebone::ModuleState>("ModuleState", 1000);
		error_state_pub = _n.advertise<beaglebone::Error>("ErrorState", 1000);

		//service servers
		position_reached_srv = _n.advertiseService("PositionReached", &module_info::position_reached, this);

		//service clients
		finished_clnt = _n.serviceClient<beaglebone::Empty>("Finished");

		//subscribers
		current_state_sub = _n.subscribe("CurrentState", 100, &module_info::currentStateCallback, this);
		velocity_sub = _n.subscribe("Velocity", 100, &module_info::velocityCallback, this);
		error_sub = _n.subscribe("Error", 100, &module_info::errorCallback, this);
		set_mode_sub = _n.subscribe("SetMode", 100, &module_info::setModeCallback, this);
		position_goal_sub = _n.subscribe("PositionGoal", 100, &module_info::positionGoalCallback, this);

		//variables
		verbose = true;     //configured with init command...
	}

	~module_info(){
	}

	bool position_reached(beaglebone::Empty::Request &req, beaglebone::Empty::Response &res ){
		//doe er iets mee, roep finished (service) aan;
		printf("Position reached...\n");
		if(!finished_clnt.call(finished_srv)){
			printf("error\n");
			Error_srv.request.nodeName = node_name;
			Error_srv.request.errorMessage = "Bad service response from Finished";
			Error_srv.request.aditionalInfo = "";
			Error_client.call(Error_srv);
			if(verbose){
				module_state_pub.publish(module_state_msg);
			}
			return false;
		}
		return true;
	}

	void positionGoalCallback(const beaglebone::Position::ConstPtr& msg){
		module_state_msg.goalX = msg->x;
		module_state_msg.goalY = msg->y;
		module_state_msg.goalTheta = msg->theta;
	}

	void setModeCallback(const beaglebone::SetMode::ConstPtr& msg){
		module_state_msg.mode = msg->mode;
	}

	void velocityCallback(const beaglebone::Velocity::ConstPtr& msg){
		module_state_msg.goalSpeed = msg->speed;
		module_state_msg.goalAngular = msg->angular;
	}

	void currentStateCallback(const beaglebone::CurrentState::ConstPtr& msg){
		module_state_msg.curX = msg->x;
		module_state_msg.curY = msg->y;
		module_state_msg.curTheta = msg->theta;
		module_state_msg.curSpeed = msg->speed;
		module_state_msg.curAngular = msg->angular;
		if(verbose){
			module_state_pub.publish(module_state_msg);
		}
	}

	void errorCallback(const beaglebone::Error::ConstPtr& msg){
		//stuur dit door naar error_state_pub;
		error_msg.errorState = msg->errorState;
		error_msg.nodeName = msg->nodeName;
		error_msg.errorMessage = msg->errorMessage;
		error_msg.aditionalInfo = msg->aditionalInfo;
		error_state_pub.publish(error_msg);
	}

	void errorHook(){
		module_state_msg.state = ERROR;
		module_state_pub.publish(module_state_msg);
	}

	void updateHook(){
		module_state_pub.publish(module_state_msg);
	}

	void configureHook(){
		//get the verbose parameter

		module_state_msg.state = IDLE;
		module_state_pub.publish(module_state_msg);
	}

	void resetHook(){
		module_state_msg.curX = 0.0;
		module_state_msg.curY = 0.0;
		module_state_msg.curTheta = 0.0;
		module_state_msg.curSpeed = 0.0;
		module_state_msg.curAngular = 0.0;
		module_state_msg.goalSpeed = 0.0;
		module_state_msg.goalAngular = 0.0;
		module_state_msg.goalX = 0.0;
		module_state_msg.goalY = 0.0;
		module_state_msg.goalTheta = 0.0;
		module_state_msg.mode = 0;
		module_state_msg.state = INIT;
		module_state_pub.publish(module_state_msg);
	}

	void startHook(){
		module_state_msg.state = RUN;
		module_state_pub.publish(module_state_msg);
	}

	void stopHook(){
		module_state_msg.state = IDLE;
		module_state_pub.publish(module_state_msg);
	}

};

int main(int argc, char ** argv){
	ros::init(argc, argv, "Module_Info");
	ros::NodeHandle n;

	module_info module(n, "module_info", 30);

	module.loop();

	return 0;
}
