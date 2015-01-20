
/*
*
* X.cpp
*
* Created on: dd/mm/yyyy
* Author: name, companyname
*/

/*
* All the includes that are being used
*/

#include <math.h>
#include <stdio.h>

#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/IMU.h"
#include "beaglebone/CurrentState.h"
#include "base_node.h"

#define PI 3.14159265

using namespace std;

class state_estimation : public base_node {

private:
ros::NodeHandle _n;
beaglebone::CurrentState pub_msg;
//ROS Subscriber
ros::Subscriber get_speed_CAN_sub;
ros::Subscriber Get_IMU_data;
//ROS Publisher
ros::Publisher set_current_state_pub;
//ROS ServiceClient

/*
* Used attributes
*/

int 	nACC,ngyr;
float speedLeft, speedRight, Theta,positionX, positionY, positionTheta,speed, angular, wheel_distance,wheel_diameter, time_step, distance,WPy, WPx, WTheta,theta,Zgyro,SPD,pyr,ay,Wxp,pxr,Wyp,ax;
float rx,ry,rt,rwx,rwy,rwt;
public:
state_estimation(ros::NodeHandle n, string node_name, int period):
base_node(node_name, n, period), _n(n)
{
	set_current_state_pub = _n.advertise<beaglebone::CurrentState>("CurrentState",1000);
	get_speed_CAN_sub = _n.subscribe("SpeedLeftRightFromCAN", 1000, &state_estimation::getSpeedCallback, this);
	Get_IMU_data =_n.subscribe("IMU_RAW", 1000, &state_estimation::get_IMUdata_Callback, this);

	speedLeft = 0.0;
	speedRight = 0.0;
	positionX = 0.0;
	positionY = 0.0;
	positionTheta = 0.0;
	speed = 0.0;
	angular = 0.0;
	wheel_distance = 0.71;
	wheel_diameter = 0.175;
	time_step = (1.0 / (float)period);
	nACC = 1;
	ngyr = 1;
	rx = 0.0;
	ry = 0.0;
	rt = 0.0;
	rwx = 0.0;
	rwy = 0.0;
	rwt = 0.0;
}
// Destructor
~state_estimation(){
}

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

void configureHook()
{
	/*
	set_current_state_pub = _n.advertise<beaglebone::CurrentState>("CurrentState",1000);
	get_speed_CAN_sub = _n.subscribe("SpeedLeftRightFromCAN", 1000, &state_estimation::getSpeedCallback, this);
	Get_IMU_data =_n.subscribe("IMU_RAW", 1000, &state_estimation::get_IMUdata_Callback, this);
	*/
	//some init value of the state estimation...
	// like wheel_distance and wheel_diameter
	double temp;
	node.getParam("wheelDistance", temp);
	//    wheel_distance = (float) temp;
	node.getParam("wheelDiameter", temp);
	//    wheel_diameter = (float) temp;
}

void resetHook()
{
	positionX = 0.0;
	positionY = 0.0;
	positionTheta = 0.0;
	speed = 0.0;
	angular = 0.0;
	speedLeft = 0.0;
	speedRight = 0.0;
	/*set_current_state_pub.shutdown();
	get_speed_CAN_sub.shutdown();*/
}

void startHook(){

}

void stopHook(){

}

//examples functions for service, callback and other functions
/*
* describe callback
*/
void getSpeedCallback(const beaglebone::SpeedLR::ConstPtr& msg)
{
/*
time_step = (1.0 / (float)period);

//from rad/s to m/s
speedLeft = msg->speedLeft * (wheel_diameter / 2);        //wl * r = Vl
speedRight = msg->speedRight * (wheel_diameter / 2);      //wr * r = Vr

//speed calculation
speed = (speedRight + speedLeft) / 2;                    //V = (Vr + Vl) / 2
//angular
angular = (speedRight - speedLeft) / wheel_distance;      //w = (Vr - Vl) / L

//position calculation without a radius calculation (speed and angular aren't zero)
if((angular < 0.001) && (angular > -0.001)){
//if(angular <= 0.0){x
//calculate the distance and position
distance = speed * time_step;                           //s = V * t

//printf("test %f * %f\n", distance, cos(positionTheta));
positionX = positionX - (distance * sin(positionTheta));             //x = -(s * sin(theta))
positionY = positionY + (distance * cos(positionTheta));             //y = s * cos(theta)
} else {
//calculate angle
positionTheta += angular * time_step;                   //Theta = w * t

//check if theta is outside the range -PI to PI
if(positionTheta > (PI)){
positionTheta -= (2 * PI);
}
if(positionTheta < -(PI)){
positionTheta += (2 * PI);
}

//implement right here the radius calculation
if((speed > 0.001) || (speed < -0.001)){
float Radius = speed / angular;                         //R = V / w
float angle = angular * time_step;                      //alpha = w * t
float tempX = Radius - (cos(angle) * Radius);           //x = R - (cos(alpha) * R)
float tempY = sin(angle) * Radius;                      //y = sin(alpha) * R
float tempDistance = sqrt(pow(tempX, 2.0) + pow(tempY, 2.0)); //pythagoras

tempDistance = (tempY < 0) ? -tempDistance : tempDistance;	//determine forward or backward

//	printf( "x: %f y: %f d: %f\n", tempX, tempY, tempDistance );

positionX -= tempDistance * sin(positionTheta);             //x = -(s * sin(theta))
positionY += tempDistance * cos(positionTheta);             //y = s * cos(theta)
}
}

// function publish
//pub_msg.x = positionX;
//pub_msg.y = positionY;
//pub_msg.theta = positionTheta;
//pufab_msg.speed = speed;
//pub_msg.angular = angular;

//publish message
//set_current_state_pub.publish(pub_msg);*/
}

void get_IMUdata_Callback(const beaglebone::IMU::ConstPtr& msg)
{
	time_step = (1.0 / (float)period); //delta t

	Zgyro=msg->Zgyr; // get gyro z range -32768 .. 32768 ~ -250 [graden/s] .. 250 [graden/s]
	ax=msg->Xacc; 	 // get accelero X
	ay=msg->Yacc;	 // get accelero Y

	ax= ((ax/16834)*9.81);//g to m/s²
	ay= ((ay/16384)*9.81);

	Zgyro/= 131; // from 32768 to 250
	Zgyro*= PI;
	Zgyro/= 180; //deg/s to rad/s

	//Zgyro = Zgyro / 2; // magic fixup factor

	rx=(ax*time_step); //snelheid robot X
	ry=(ay*time_step); //snelheid robot y

	speed=rx; //snelheid robot

	angular=(Zgyro); // hoeksnelheid

	if ((angular > -0.01)||(angular < -0.04))
	{
	   positionTheta+=(angular * time_step); //hoek
	}

	if(positionTheta > (PI))
	{
	   positionTheta -= (2 * PI);
	}

	if(positionTheta < -(PI))
	{
	   positionTheta += (2 * PI);
	}

	if ((speed > 0.01)||(speed < -0.01))
	{
	positionX+=((cos(positionTheta)*rx)+(sin(positionTheta)*ry)*time_step); //
	positionY+=((sin(positionTheta)*rx)+(cos(positionTheta)*ry)*time_step); //
	}

	pub_msg.x = positionX;
	pub_msg.y = positionY;
	pub_msg.theta = positionTheta;
	pub_msg.speed = speed;
	pub_msg.angular = angular;

	set_current_state_pub.publish(pub_msg);
}

};
/*
* describe service
*/
/*
bool testService(std_msgs::string::Request& req, std_msgs::string::Response& res)
{
return true;
}
*/
int main(int argc, char ** argv){
ros::init(argc, argv, "State_Estimation");
ros::NodeHandle n;

state_estimation state(n, "state_estimation", 30);

state.loop();

return 0;
}
/*
#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/CurrentState.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265

//include base class
#include "Beaglebone/base_node.h"

using namespace std;

class state_estimation : public base_node {

private:
ros::NodeHandle _n;
ros::Publisher set_current_state_pub;
ros::Subscriber get_speed_CAN_sub;
beaglebone::CurrentState pub_msg;
float speedLeft, speedRight, positionX, positionY, positionTheta, speed, angular, wheel_distance, wheel_diameter, time_step, distance;

public:
state_estimation(ros::NodeHandle n, string node_name, int period):
base_node(node_name, n, period), _n(n)
{
speedLeft = 0.0;
speedRight = 0.0;
positionX = 0.0;
positionY = 0.0;
positionTheta = 0.0;
speed = 0.0;
angular = 0.0;
wheel_distance = 0.71;
wheel_diameter = 0.175;
time_step = (1.0 / (float)period);

}

~state_estimation(){
}

void getSpeedCallback(const beaglebone::SpeedLR::ConstPtr& msg){
time_step = (1.0 / (float)period);

//from rad/s to m/s
speedLeft = msg->speedLeft * (wheel_diameter / 2);        //wl * r = Vl
speedRight = msg->speedRight * (wheel_diameter / 2);      //wr * r = Vr

//speed calculation
speed = (speedRight + speedLeft) / 2;                     //V = (Vr + Vl) / 2
//angular
angular = (speedRight - speedLeft) / wheel_distance;      //w = (Vr - Vl) / L

//position calculation without a radius calculation (speed and angular aren't zero)
if((angular < 0.001) && (angular > -0.001)){
//if(angular <= 0.0){
//calculate the distance and position
distance = speed * time_step;                           //s = V * t

//printf("test %f * %f\n", distance, cos(positionTheta));
positionX = positionX - (distance * sin(positionTheta));             //x = -(s * sin(theta))
positionY = positionY + (distance * cos(positionTheta));             //y = s * cos(theta)
} else {
//calculate angle
positionTheta += angular * time_step;                   //Theta = w * t

//check if theta is outside the range -PI to PI
if(positionTheta > (PI)){
positionTheta -= (2 * PI);
}
if(positionTheta < -(PI)){
positionTheta += (2 * PI);
}

//implement right here the radius calculation
if((speed > 0.001) || (speed < -0.001)){
float Radius = speed / angular;                         //R = V / w
float angle = angular * time_step;                      //alpha = w * t
float tempX = Radius - (cos(angle) * Radius);           //x = R - (cos(alpha) * R)
float tempY = sin(angle) * Radius;                      //y = sin(alpha) * R
float tempDistance = sqrt(pow(tempX, 2.0) + pow(tempY, 2.0)); //pythagoras

tempDistance = (tempY < 0) ? -tempDistance : tempDistance;	//determine forward or backward

//	printf( "x: %f y: %f d: %f\n", tempX, tempY, tempDistance );

positionX -= tempDistance * sin(positionTheta);             //x = -(s * sin(theta))
positionY += tempDistance * cos(positionTheta);             //y = s * cos(theta)
}
}

//dummy function publish
pub_msg.x = positionX;
pub_msg.y = positionY;
pub_msg.theta = positionTheta;
pub_msg.speed = speed;
pub_msg.angular = angular;

//publish message
set_current_state_pub.publish(pub_msg);
}

void errorHook(){
}

void updateHook(){
}

void configureHook(){
set_current_state_pub = _n.advertise<beaglebone::CurrentState>("CurrentState",1000);
get_speed_CAN_sub = _n.subscribe("SpeedLeftRightFromCAN", 1000, &state_estimation::getSpeedCallback, this);


//some init value of the state estimation...
// like wheel_distance and wheel_diameter
double temp;
node.getParam("wheelDistance", temp);
//    wheel_distance = (float) temp;
node.getParam("wheelDiameter", temp);
//    wheel_diameter = (float) temp;

}

void resetHook(){
positionX = 0.0;
positionY = 0.0;
positionTheta = 0.0;
speed = 0.0;
angular = 0.0;
speedLeft = 0.0;
speedRight = 0.0;
set_current_state_pub.shutdown();
get_speed_CAN_sub.shutdown();
}

void startHook(){
}

void stopHook(){
}

};

int main(int argc, char ** argv){
ros::init(argc, argv, "State_Estimation");
ros::NodeHandle n;

state_estimation state(n, "state_estimation", 30);

state.loop();

return 0;
}
*/

