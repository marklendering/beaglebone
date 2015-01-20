#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/CurrentState.h"
#include "beaglebone/Position.h"
#include "beaglebone/Empty.h"
#include "beaglebone/Command.h"
#include "beaglebone/SetPosition.h"
#include "beaglebone/SetMode.h"

#include <math.h>

#define PI 3.14159265
#define STOP 1

//include base class
#include "base_node.h"

using namespace std;

class motion_planner : public base_node {

private:
	ros::NodeHandle _n;
	ros::Subscriber current_state_sub, set_mode_sub;
	ros::Publisher set_points_pub, position_goal_pub;
	ros::ServiceClient position_reached_clnt, command_clnt;
	ros::ServiceServer set_position_srv, done_position_srv;
	beaglebone::Position set_points_msg;
	beaglebone::Position position_goal_msg;
	beaglebone::Empty position_reached_srv;
	beaglebone::Command command_srv;

	ros::Duration delay;

	bool calculated_new_setpoint;
	int Mode;

	float goalX, goalY, goalTheta, curX, curY, curTheta, gGoalX, gGoalY;
	float path[3][3];                                  //first array is the pathnumber, the second x, y, theta
	int pathnumber;

public:
	motion_planner(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
		current_state_sub = _n.subscribe("CurrentState", 1000, &motion_planner::currentStateCallback, this);
		set_mode_sub = _n.subscribe("SetMode", 1000, &motion_planner::setModeCallback, this);
		position_goal_pub = _n.advertise<beaglebone::Position>("PositionGoal", 1000);
		set_points_pub = _n.advertise<beaglebone::Position>("SetPoints", 1000);
		position_reached_clnt = _n.serviceClient<beaglebone::Empty>("PositionReached");
		command_clnt = _n.serviceClient<beaglebone::Command>("Command");
		set_position_srv = _n.advertiseService("SetPosition", &motion_planner::SetPosition, this);
		done_position_srv = _n.advertiseService("DonePosition", &motion_planner::DonePosition, this);

		goalX = 0.0;
		goalY = 0.0;
		goalTheta = 0.0;
		curX = 0.0;
		curY = 0.0;
		curTheta = 0.0;
		pathnumber = 0;
		calculated_new_setpoint = false;
		delay = ros::Duration(1);
	}

	~motion_planner(){
	}

	bool SetPosition(beaglebone::SetPosition::Request &req, beaglebone::SetPosition::Response &res){
		//goal position calculation, given coordinate + current coordinates
		goalX += sin(goalTheta) * req.y + cos(goalTheta) * req.x;        //= req.x + curX;
		goalY += cos(goalTheta) * req.y + sin(goalTheta) * req.x;        //= req.y + curY;
		goalTheta += req.theta;//= req.theta + curTheta;

		printf("goal: (%f, %f, %f)\n", goalX, goalY, goalTheta);

		while (goalTheta < -PI){
			goalTheta += 2*PI;
		}
		while (goalTheta > PI){
			goalTheta -= 2*PI;
		}

		//call PositionGoal (to module info)
		position_goal_msg.x = goalX;
		position_goal_msg.y = goalY;
		position_goal_msg.theta = goalTheta;
		position_goal_pub.publish(position_goal_msg);

		calculated_new_setpoint = false;
		pathnumber = 0;
		//printf("Motion_Planner: Got new position! (%f,%f,%f)\n", goalX, goalY, goalTheta);

		return true;
	}

	bool DonePosition(beaglebone::Empty::Request &req, beaglebone::Empty::Response &res){
		//printf("Motion_Planner: Got signal, finished path: %d\n", pathnumber);

		//check for the next path
		if(pathnumber >=3){
			pathnumber = 0;

			//set state to idle
			command_srv.request.state = 1;
			if(!command_clnt.call(command_srv)){
				Error_srv.request.nodeName = node_name;
				Error_srv.request.errorMessage = "Couldn't call a stop command";
				Error_srv.request.aditionalInfo = "";
				Error_client.call(Error_srv);
			}

			//if position is reached call PositionReached
			if(position_reached_clnt.call(position_reached_srv)){
				return true;
			}
			else {
				return false;
			}
		}

		delay.sleep();

		set_points_msg.x = path[pathnumber][0];
		set_points_msg.y = path[pathnumber][1];
		set_points_msg.theta = path[pathnumber][2];

		set_points_pub.publish(set_points_msg);
		pathnumber++;

		return true;
	}

	void setModeCallback(const beaglebone::SetMode::ConstPtr& msg){
		Mode = msg->mode;
	}

	void currentStateCallback(const beaglebone::CurrentState::ConstPtr& msg){
		//get currentposition
		curX = msg->x;
		curY = msg->y;
		curTheta = msg->theta;
	}

	void errorHook(){
	}

	void updateHook(){
		if(Mode == POSITION){
			//printf("Calculated new setpoint: %d\n", calculated_new_setpoint);
			if(!calculated_new_setpoint){
				//printf("Calculating new setpoint (N: %d)\n", pathnumber);
				//calculate path 1,2 and 3

				//path 1:
				//calculate approach angle
				float errorX = goalX - curX;
				float errorY = goalY - curY;
				float approachAngle;

				if(((errorY >= -0.01) && (errorY <= 0.01)) && ((errorX >= -0.01) && (errorX <= 0.01))){
					//no movement, go directly to goal theta
					approachAngle = goalTheta;
				}
				else {
					//calculate approachAngle with the 4qwadrant with reference on the Y axe ipv X axe (- 1/4 PI)
					approachAngle = atan2(errorY, errorX) - (0.5 * PI);
				}

				//correct the -PI -> PI range

				if(approachAngle <= -PI){
					approachAngle += (2 * PI);
				} else if(approachAngle > PI){
					approachAngle -= (2 * PI);
				}

				//printf("Approach angle: %f\n", approachAngle);

				//fill path 1:
				path[0][0] = curX;                  //fill x
				path[0][1] = curY;                  //fill y
				path[0][2] = approachAngle;         //fill theta

				//path 2:
				//path 2 will be filled with the goalposition (x,y)
				path[1][0] = goalX;
				path[1][1] = goalY;
				path[1][2] = approachAngle;

				//path 3:
				//path 3 is filled with the goaltheta
				path[2][0] = goalX;
				path[2][1] = goalY;
				path[2][2] = goalTheta;

				//printf("paths:\n\t(%f,%f,%f)\n\t(%f,%f,%f)\n\t(%f,%f,%f)\n", path[0][0],path[0][1],path[0][2],path[1][0],path[1][1],path[1][2],path[2][0],path[2][1],path[2][2]);

				//calculated_new_setpoint = true;

				set_points_msg.x = path[pathnumber][0];
				set_points_msg.y = path[pathnumber][1];
				set_points_msg.theta = path[pathnumber][2];

				set_points_pub.publish(set_points_msg);
				pathnumber++;
				calculated_new_setpoint = true;
			}
		}
	}

	void configureHook(){
		//some init value of the motion planner...
	}

	void resetHook(){
		//reset variables
		calculated_new_setpoint = false;
		goalX = 0.0;
		goalY = 0.0;
		goalTheta = 0.0;
		curX = 0.0;
		curY = 0.0;
		curTheta = 0.0;
		pathnumber = 0;

	}

	void startHook(){
	}

	void stopHook(){
	}

};

int main(int argc, char ** argv){
  ros::init(argc, argv, "Motion_Planner");
  ros::NodeHandle n;

  motion_planner motion(n, "motion_planner", 30);

  motion.loop();

  return 0;
}


