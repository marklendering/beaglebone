#include "ros/ros.h"

#include "beaglebone/Velocity.h"
#include "beaglebone/CurrentState.h"
#include "beaglebone/Position.h"
#include "beaglebone/Empty.h"
#include "beaglebone/SetMode.h"


#include <math.h>

#define PI 3.14159265

#define TURNING 0
#define STRAIGHT 1
#define DEFAULT 2

#define MIN_SPEED 0.2
#define MAX_SPEED 1.2
#define NON_SPEED 0.01

#define MIN_ANGULAR 0.4
#define MAX_ANGULAR 2.0

#define XY_MARGIN 0.05
#define T_MARGIN 0.05

//include base class
#include "base_node.h"

using namespace std;

class controller : public base_node {

private:
	ros::NodeHandle _n;
	ros::Subscriber current_state_sub, set_points_sub, set_mode_sub;
	ros::Publisher velocity_pub;
	ros::ServiceClient done_position_clnt;
	beaglebone::Velocity velocity_msg;
	beaglebone::Empty done_position_srv;
	float speed, angular, goalX, goalY, goalTheta, curX, curY, curTheta, pGainAngular, iGainAngular, pGainSpeed, iGainSpeed, iError, errorX, errorY, errorTheta;
	bool finished, reached_x, reached_y, reached_theta, obstructed;
	int Mode, modus;
	//float min_speed, max_speed, min_angular, max_angular

public:
	controller(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
		current_state_sub = _n.subscribe("CurrentState", 1000, &controller::currentStateCallback, this);
		set_points_sub = _n.subscribe("SetPoints", 1000, &controller::setPointsCallback, this);
		set_mode_sub = _n.subscribe("SetMode", 1000, &controller::setModeCallback, this);
		velocity_pub = _n.advertise<beaglebone::Velocity>("Velocity",1000);
		done_position_clnt = _n.serviceClient<beaglebone::Empty>("DonePosition");

		goalX = 0.0;
		goalY = 0.0;
		goalTheta = 0.0;
		speed = 0.1;
		angular = 1.5;
		finished = true;
		curX = 0.0;
		curY = 0.0;
		curTheta = 0.0;
		pGainAngular = 1.0;
		iGainAngular = 0.05;
		pGainSpeed = 0.30;
		iGainSpeed = 0.02;
		errorX = 0.0;
		errorY = 0.0;
		errorTheta = 0.0;
		iError = 0.0;

		obstructed = false;

		double temp;
		temp = (double) pGainAngular;
		_n.setParam("pGainAngular", temp);
		temp = (double) iGainAngular;
		_n.setParam("iGainAngular", temp);
		temp = (double) pGainSpeed;
		_n.setParam("pGainSpeed", temp);
	}

	~controller(){
	}

	float limit(float n, float min, float max){
		return (n < min) ? min : ((n > max) ? max : n);
	}

	bool isWithinMargin(float n, float m){
		return (n >= -m) && (n <= m);
	}

	void setPointsCallback(const beaglebone::Position::ConstPtr& msg){
		float tempx = goalX;
		float tempy = goalY;
		float temptheta = goalTheta;

		//printf("temptheta: %f \n", temptheta);

		//dummy function

		//dummy function publish
		//get goal position
		goalX = msg->x;
		goalY = msg->y;
		goalTheta = msg->theta;
		finished = false;

		reached_x = reached_y = reached_theta = false;

		//	printf("gX: %f gY: %f gT: %f\n", goalX, goalY, goalTheta);
		//	printf("tX: %f tY: %f tT: %f\n", tempx, tempy, temptheta);

		//    errorTheta = goalTheta - curTheta + gErrorTheta;

		//if((tempx == goalX) && (tempy == goalY)){
		if(isWithinMargin(tempx-goalX, 2*XY_MARGIN) && isWithinMargin(tempy-goalY,2*XY_MARGIN)){
			modus = TURNING;
			//	printf("TURNING\n");
		}
		else { //if(temptheta == goalTheta) {
			modus = STRAIGHT;
			//	printf("STRAIGTH\n");
			//errorX = goalX - curX;
			//errorY = goalY - curY;
			//} else {
			//	printf("DEFAULT\n");
			//	modus = DEFAULT;
		}
	}

	void setModeCallback(const beaglebone::SetMode::ConstPtr& msg){
		//    printf("Got Mode...\n");
		Mode = msg->mode;
	}

	void currentStateCallback(const beaglebone::CurrentState::ConstPtr& msg){
		//    printf("Got current state...\n");
		//get currentposition
		curX = msg->x;
		curY = msg->y;
		curTheta = msg->theta;
	}



	void errorHook(){
	}

	void updateHook(){
		//    printf("Mode: %d\n", Mode);
		if(Mode == POSITION){
			//regellus

			if(finished){				//check if goal is reached or obstructed
				velocity_msg.speed = 0.0;					//send velocity zero
				velocity_msg.angular = 0.0;

				velocity_pub.publish(velocity_msg);
			}
			else{											//if not finished

				//if (modus == TURNING)
				//{
				//calculate errorTheta (goalTheta - curTheta)
				errorTheta = goalTheta - curTheta;
				errorX = goalX - curX;
				errorY = goalY - curY;

				//errorTheta = fmod(errorTheta, PI); // limit to -PI - PI range

				if(errorTheta < -PI){
					errorTheta += 2 * PI;
				} else if(errorTheta > PI){
					errorTheta -= 2 * PI;
				}

				//check boundaries ( -0.1 > errorTheta > 0.1 )
				if(((errorTheta < T_MARGIN) && (errorTheta > -T_MARGIN))){
					errorTheta = 0;
					reached_theta = true;
					iError = 0.0;
				}
				else{
					reached_theta = false;
				}
				
				iError += errorTheta;
				iError = limit(iError, -1000, 1000);

				//		if(iError > 1000){
				//			iError = 1000;
				//		} else if (iError < -1000){
				//			iError = -1000;
				//		}

				//calculate angular velocity (Kr * (errorTheta))
				angular = (pGainAngular * errorTheta) + (iGainAngular * iError);

				if(errorTheta > 0)
					angular = limit(angular, MIN_ANGULAR, MAX_ANGULAR);
				else if(errorTheta < 0)
					angular = limit(angular, -MAX_ANGULAR, -MIN_ANGULAR);

				//}
				//else

				if(modus == STRAIGHT || (errorX > 0.5 || errorX < -0.5 || errorY > 0.5 || errorY < -0.5)){
					//if(errorX > 0.5 || errorX < -0.5 || errorY > 0.5 || errorY < -0.5)
					//	printf("Error is too big! (%f,%f)\n", errorX, errorY);

					//calculate errorX (goalX - curX)
					//errorX = goalX - curX;
					//errorY = goalY - curY;

					///////////////////////////////////////////////////////////////////////////
					float tempGoalTheta = atan2(errorY, errorX) - 0.5*PI;
					if(tempGoalTheta < -PI)
						tempGoalTheta += 2*PI;
					else if(tempGoalTheta > PI)
						tempGoalTheta -= 2*PI;

					//printf("GT: %f CT: %f\n", tempGoalTheta, curTheta);

					//calculate errorTheta (goalTheta - curTheta)
							float tempErrorTheta = tempGoalTheta - curTheta;
							//errorTheta = fmod(errorTheta, PI); // limit to -PI - PI range

							if(tempErrorTheta < -PI){
									tempErrorTheta += 2 * PI;
							} else if(tempErrorTheta > PI){
									tempErrorTheta -= 2 * PI;
							}

					//		printf( "Error: %f\n", tempErrorTheta );

					/*
					if ((tempErrorTheta > 0.25 || tempErrorTheta < -0.25) && !(isWithinMargin(errorX, XY_MARGIN) && isWithinMargin(errorY, XY_MARGIN)))
					{
						errorTheta = tempErrorTheta;
						iError += errorTheta;
						iError = limit(iError, -1000, 1000);
						//iError = (iError > 1000) ? 1000 : ((iError < -1000) ? -1000 : iError);
						angular = pGainAngular * errorTheta + iGainAngular * iError;
						if (errorTheta > 0)
							angular = limit(angular, MIN_ANGULAR, MAX_ANGULAR);
						else if (errorTheta < 0)
							angular = limit(angular, -MAX_ANGULAR, -MIN_ANGULAR);
						errorX = errorY = 0; // Stop the vehicle to just turn
						//printf("angular: %f integral: %f\n", angular, iError);
					}
					*/
					/////////////////////////////////////////////////////////////////////////////////////

					//check boundaries (-0.01 > errorX > 0.01)
					//if(((errorX < XY_MARGIN) && (errorX > -XY_MARGIN))){
					reached_x = (isWithinMargin(errorX, XY_MARGIN));
					  //errorX = 0;

					//check boundaries (-0.1 > errorX > 0.1)
					//if(((errorY < XY_MARGIN) && (errorY > -XY_MARGIN))){
					reached_y = (isWithinMargin(errorY, XY_MARGIN));
					  //errorY = 0;

					//calculate speed velocity (Kp * (wortel((errorX)^2 + (errorY)^2))
					speed = (pGainSpeed * sqrt(pow(errorX, 2.0) + pow(errorY, 2.0)));

					if ((tempErrorTheta > 0.25 || tempErrorTheta < -0.25) && !(reached_x && reached_y))
					{
							errorTheta = tempErrorTheta;
							iError += errorTheta;
							iError = limit(iError, -1000, 1000);
							//iError = (iError > 1000) ? 1000 : ((iError < -1000) ? -1000 : iError);
							angular = pGainAngular * errorTheta + iGainAngular * iError;
							if (errorTheta > 0)
									angular = limit(angular, MIN_ANGULAR, MAX_ANGULAR);
							else if (errorTheta < 0)
									angular = limit(angular, -MAX_ANGULAR, -MIN_ANGULAR);
							speed = 0; // Slowdown the vehicle to turn
							//printf("angular: %f integral: %f\n", angular, iError);
					}

					//limit the speed velocity
					if(speed > MAX_SPEED){
						speed = MAX_SPEED;
					}else if((speed < MIN_SPEED) && (speed > NON_SPEED)) {
						speed = MIN_SPEED;
					}

					//	printf("Speed: %f Angular: %f\n", speed, angular);

					// TODO drive backwards in case of overshoot!

				} else { // If not in STRAIGHT mode
					//errorX = 0;
					//errorY = 0;
					speed = 0.0;
					reached_x = reached_y = true;
				}
				//	if (speed > 0.45)
					printf( "Error x: %f\nError y: %f\nError t: %f\nspeed: %f\nangular: %f\n", errorX, errorY, errorTheta, speed, angular);


				//publish velocity
				velocity_msg.speed = speed;
				velocity_msg.angular = angular;
				velocity_pub.publish(velocity_msg);

				// Check if goal is reached
				if((reached_x) && (reached_y) && (reached_theta)){


					//then send a done position message

					// and a zero velocity
					velocity_msg.speed = 0.0;
					velocity_msg.angular = 0.0;
					velocity_pub.publish(velocity_msg);

					iError = 0.0;

					finished = true;

					if(done_position_clnt.call(done_position_srv)){
						// Finished call was succesfull
					}
				}
			}
		}
	}

	void configureHook(){

		double temp;

		_n.getParam("pGainAngular", temp);
		pGainAngular = (float) temp;
		_n.getParam("iGainAngular", temp);
		iGainAngular = (float) temp;
		_n.getParam("pGainSpeed", temp);
		pGainSpeed = (float) temp;
		//some init value of the state estimation...
	}

	void resetHook(){
		//reset variables
		goalX = 0.0;
		goalY = 0.0;
		goalTheta = 0.0;
		speed = 0.1;
		angular = 1.5;
		curX = 0.0;
		curY = 0.0;
		curTheta = 0.0;
		finished = true;
		pGainAngular = 1.0;
		iGainAngular = 0.01;
		pGainSpeed = 0.5;
		iGainSpeed = 0.0;
		errorX = 0.0;
		errorY = 0.0;
		errorTheta = 0.0;
		iError = 0.0;
	}

	void startHook(){
	}

	void stopHook(){
	}

};

int main(int argc, char ** argv){
  ros::init(argc, argv, "Controller");
  ros::NodeHandle n;

  controller control(n, "controller", 30);

  control.loop();

  return 0;
}
