#ifndef CONTROL_FLOW
#define CONTROL_FLOW

#include "ros/ros.h"
#include "beaglebone/OperationState.h"
#include "beaglebone/AppendError.h"
#include "beaglebone/Command.h"
#include "beaglebone/Init.h"
#include "beaglebone/Mode.h"
#include "beaglebone/SetPosition.h"
#include "beaglebone/Error.h"
#include "beaglebone/Position.h"
#include "beaglebone/Velocity.h"
#include "beaglebone/SetMode.h"
#include <string.h>

using namespace std;

/* Commands */
#define START  0
#define STOP   1
#define RESET  2

/* Operation States */
#define INIT  0
#define IDLE  1
#define RUN   2
#define ERROR 3

/* modus */
#define VELOCITY 0
#define POSITION 1

/* Error States */
#define NO_ERROR		0
#define NO_CAN_NETWORK		1
#define NO_SANITY		2
#define MBED_ERROR		3 //should be more defined...
#define NO_VALID_SETPOINT	4
#define NO_VALID_VELOCITY	5
#define MBED_TIMEOUT		6
#define HIT_WALL		7

/**********************************************
 * Base class of all nodes in the beaglebone. *
 * Is used to handle the states of the nodes. *
 * Also the ROS intitialize is done in here.  *
 **********************************************/

class control_flow {

private:
	string node_name;
	bool verbose;
	int State;
	int ErrorState;
	int period;
	int mode;

	ros::Rate loop_rate;

	int getState(){
		return State;
	}

	ros::NodeHandle node;
	ros::ServiceClient SetPosition_client, Error_client;
	ros::ServiceServer Mode_server, Init_server, Command_server;
	ros::Subscriber Error_sub, GoalVelocity_sub, GoalPosition_sub;
	ros::Publisher OperationState_pub, Velocity_pub, SetMode_pub;

	beaglebone::SetMode SetMode_msg;
	beaglebone::OperationState OperationState_msg;
	beaglebone::Velocity Velocity_msg;
	beaglebone::SetPosition SetPosition_srv;
	beaglebone::AppendError Error_srv;

	/* standard State methods, applying for every node */
	void init(void);
	void idle(void);
	void run(void);
	void error(void);

public:
	/* Constructor, assigning basic topic (states) */
	control_flow(string name, ros::NodeHandle n, int period);
	
	/* Destructor */
	virtual ~control_flow(){};

	/* callback for topic GoalVelocity from the OS, to get the goalVelocity */
	void GoalVelocity_callback(const beaglebone::Velocity::ConstPtr& msg);

	/* callback for topic GoalPosition from the OS, to get the goalposition */
	void GoalPosition_callback(const beaglebone::Position::ConstPtr& msg);

	/* callback for topic Error from Error Handler, to get an error command */
	void Error_callback(const beaglebone::Error::ConstPtr& msg);

	/* service Command, to get command reguests from the OS */
	bool Command(beaglebone::Command::Request &req,
						beaglebone::Command::Response &res);

	/* service Init, to get initialize reguests from the OS with parameters */
	bool Init(beaglebone::Init::Request &req,
					 beaglebone::Init::Response &res);

	/* service Mode, to get mode reguests from the OS (velocity or position control) */
	bool Mode(beaglebone::Mode::Request &req,
					 beaglebone::Mode::Response &res);
	
	/* State methods, should be overritten at every node to preform node spefic calculations */
	void errorHook();
	void updateHook();
	void configureHook();
	void startHook();
	void stopHook();
	void resetHook();
	void loop(void);
};

#endif  //CONTROL_FLOW
