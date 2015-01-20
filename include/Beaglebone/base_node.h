#ifndef BASE_NODE
#define BASE_NODE

#include "ros/ros.h"
#include "Beaglebone/OperationState.h"
#include "Beaglebone/AppendError.h"
#include <string.h>

using namespace std;

/* Operation States */
#define INIT  0
#define IDLE  1
#define RUN   2
#define ERROR 3

/* Error States */
#define NO_ERROR		0
#define NO_CAN_NETWORK		1
#define NO_SANITY		2
#define MBED_ERROR		3 //should be more defined...
#define NO_VALID_SETPOINT	4
#define NO_VALID_VELOCITY	5
#define MBED_TIMEOUT		6
#define HIT_WALL		7

/* Modus */
#define VELOCITY     0
#define POSITION     1

/**********************************************
 * Base class of all nodes in the beaglebone. *
 * Is used to handle the states of the nodes. *
 * Also the ROS intitialize is done in here.  *
 **********************************************/

class base_node {

protected:
	ros::NodeHandle node;
	ros::Subscriber OperationState_sub;
	ros::Subscriber ErrorState_sub;
	
	bool verbose;
	int State;
	int ErrorState;
	int period;

	ros::Rate loop_rate;

	int getState(){
		return State;
	}

private:
	/* standard State methods, applying for every node */
	void init(void);
	void idle(void);
	void run(void);
	void error(void);

 public:
	ros::ServiceClient Error_client;
	Beaglebone::AppendError Error_srv;
	string node_name;

	/* Constructor, assigning basic topic (states) */
	base_node(string name, ros::NodeHandle n, int period);
	
	virtual ~base_node(){};

	/* callback for topic OperationState, to get the Operation State */
	void OperationState_callback(const Beaglebone::OperationState::ConstPtr& msg);

	/* State methods, should be overritten at every node to preform node spefic calculations */
	virtual void errorHook() = 0;
	virtual void updateHook() = 0;
	virtual void configureHook() = 0;
	virtual void startHook() = 0;
	virtual void stopHook() = 0;
	virtual void resetHook() = 0;
	void loop(void);
};

#endif  //BASE_NODE
