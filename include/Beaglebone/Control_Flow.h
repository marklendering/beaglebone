#ifndef CONTROL_FLOW
#define CONTROL_FLOW

#include "ros/ros.h"
#include "Beaglebone/OperationState.h"
#include "Beaglebone/AppendError.h"
#include "Beaglebone/Command.h"
#include "Beaglebone/Init.h"
#include "Beaglebone/Mode.h"
#include "Beaglebone/SetPosition.h"
#include "Beaglebone/Error.h"
#include "Beaglebone/Position.h"
#include "Beaglebone/Velocity.h"
#include "Beaglebone/SetMode.h"
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

	Beaglebone::SetMode SetMode_msg;
	Beaglebone::OperationState OperationState_msg;
	Beaglebone::Velocity Velocity_msg;
	Beaglebone::SetPosition SetPosition_srv;
	Beaglebone::AppendError Error_srv;

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
	void GoalVelocity_callback(const Beaglebone::Velocity::ConstPtr& msg);

	/* callback for topic GoalPosition from the OS, to get the goalposition */
	void GoalPosition_callback(const Beaglebone::Position::ConstPtr& msg);

	/* callback for topic Error from Error Handler, to get an error command */
	void Error_callback(const Beaglebone::Error::ConstPtr& msg);

	/* service Command, to get command reguests from the OS */
	bool Command(Beaglebone::Command::Request &req,
						Beaglebone::Command::Response &res);

	/* service Init, to get initialize reguests from the OS with parameters */
	bool Init(Beaglebone::Init::Request &req,
					 Beaglebone::Init::Response &res);

	/* service Mode, to get mode reguests from the OS (velocity or position control) */
	bool Mode(Beaglebone::Mode::Request &req,
					 Beaglebone::Mode::Response &res);
	
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
