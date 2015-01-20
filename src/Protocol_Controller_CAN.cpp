
/*
*
* Protocol_Controller_CAN.cpp
*
* Created on: dd/mm/yyyy
* Author: name, companyname
*/

/*
* All the includes that are being used
*/
#include "base_node.h"
#include "Protocol_Controller_CAN.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "beaglebone/SpeedLR.h"
#include "beaglebone/AppendError.h"
#include "beaglebone/IMU.h"
#include "beaglebone/LRS.h"

using namespace std;
/* create float from two bytes and chek if negative only for 2's complement!!  */

float create_float_from_bytes( char *buf, int offset)
{
unsigned char b0, b1;
b0 = buf[offset];
b1 = buf[offset+1];
unsigned short total = (b0 << 8) | b1; // Ensure endianness!

if((total&0x8000)==0x8000)					//chek bit 0x8000 for negative digit
{
total = ((total xor 0xFFFF)+1);
total *= -1;
}

signed short stotal = total; // convert to signed

return (float)stotal;

}


class protocol_controller_CAN : public base_node {

private:
ros::NodeHandle _n;
CAN_Driver* CAN_driver;

//ROS Subscriber
ros::Subscriber subSetSpeed;
ros::Subscriber set_speed_sub;

//ROS Publisher
ros::Publisher get_speed_pub;
ros::Publisher LRS_RAW_pub;
ros::Publisher IMU_RAW_pub;

//ROS ServiceServer

//ROS ServiceClient


/*
* Used attributes
*/
int i;
float pGain, iGain, dGain;
beaglebone::LRS LRS_msg;
bool LRS_ran_flag;
bool LRS_int_flag;

public:
protocol_controller_CAN(ros::NodeHandle n, string node_name, int period):
base_node(node_name, n, period), _n(n)
{
CAN_driver = new CAN_Driver();

if(!CAN_driver->open_CAN((char *)"can0"))
{
	Error_srv.request.nodeName = node_name;
	Error_srv.request.errorMessage = "CAN: setting up network";
	Error_srv.request.aditionalInfo = "";
	Error_client.call(Error_srv);
	perror("Couldn't open the CAN interface");
}

//advertise <Package::msg>("Topicname", buffersize)
//define the subcribers and publishers
set_speed_sub = _n.subscribe("SpeedLeftRightToCAN", 100, &protocol_controller_CAN::setSpeedCallback, this);

get_speed_pub = _n.advertise<beaglebone::SpeedLR>("SpeedLeftRightFromCAN", 100);
IMU_RAW_pub   = _n.advertise<beaglebone::IMU>("IMU_RAW", 100);
LRS_RAW_pub   = _n.advertise<beaglebone::LRS>("LRS_RAW", 100);

pGain = 0.05;
iGain = 0.0015;
dGain = 0.0;

double pgain = (double) pGain;
double igain = (double) iGain;
double dgain = (double) dGain;

_n.setParam("pGain", pgain);
_n.setParam("iGain", igain);
_n.setParam("dGain", dgain);
}

// Destructor
~protocol_controller_CAN(){
delete CAN_driver;
}

// State methods that are executed at the loop rate when the node is in a specific state
void init(){
errorPolling();
}

void idle(){
errorPolling();
}

void run()
{
}

void error()
{
}

// state transition methods that are executed once when changing to a state
void errorHook(){
set_speed_sub.shutdown();
}

void updateHook(){
errorPolling();
ask_speed(MCLEFT);
can_get();
ask_speed(MCRIGHT);
can_get();
ask_IMU(REQ_ACC);
can_get();
ask_IMU(REQ_GYR);
can_get();

for (int k=0;k<9;k++){
ask_LRS();
can_get();
can_get();
}
can_get();
}

void configureHook(){
LRS_ran_flag = false;
LRS_int_flag = false;

char buf[6];

double pgain;
double igain;
double dgain;

_n.getParam("pGain",  pgain);
_n.getParam("iGain",  igain);
_n.getParam("dGain",  dgain);

pGain = (float) pgain;
iGain = (float) igain;
dGain = (float) dgain;

// init to Motor Controllers
buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_PGAIN;
converter.Float =  pGain; //some init value
for(i = 0; i<4; i++){
buf[i+2] = (converter.Int >> (8*i)) & 0xff;
}
CAN_driver->write_CAN(buf, 6, ALLMBEDS); //write P gain to all Motor Controllers

buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_IGAIN;
converter.Float = iGain; //some init value
for(i = 0; i<4; i++){
buf[i+2] = (converter.Int >> (8*i)) & 0xff;
}
CAN_driver->write_CAN(buf, 6, ALLMBEDS); //write I gain to all Motor Controllers

buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_DGAIN;
converter.Float = dGain; //some init value
for(i = 0; i<4; i++){
buf[i+2] = (converter.Int >> (8*i)) & 0xff;
}
CAN_driver->write_CAN(buf, 6, ALLMBEDS); //write D gain to all Motor Controllers


//configure controller (Motor Controller goes to idle)
buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_CONFIGURE_CONTROLLER;
if(!CAN_driver->write_CAN(buf, 2, ALLMBEDS)){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: write error";
Error_srv.request.aditionalInfo = "all motor controllers";
Error_client.call(Error_srv);
}
}

void resetHook(){
/*
get_speed_pub.shutdown();
if(set_speed_sub != NULL)
set_speed_sub.shutdown();
*/
char buf[2];
//reset Motor Controllers

buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_RESET_CONTROLLER;

if(!CAN_driver->write_CAN(buf, 2, ALLMBEDS))
{ //reset controller to all Motor Controllers
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: write error";
Error_srv.request.aditionalInfo = "all motor controllers";
Error_client.call(Error_srv);
}
}

void startHook(){
char buf[2];

buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_START_CONTROLLER;

if(!CAN_driver->write_CAN(buf, 2, ALLMBEDS)){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: write error";
Error_srv.request.aditionalInfo = "all motor controllers";
Error_client.call(Error_srv);
}
}

void stopHook(){
char buf[2];

buf[0] = COMMAND_WRITE;
buf[1] = COMMAND_STOP_CONTROLLER;

if(!CAN_driver->write_CAN(buf, 2, ALLMBEDS)){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: write error";
Error_srv.request.aditionalInfo = "all motor controllers";
Error_client.call(Error_srv);
}
}

//examples functions for service, callback and other functions
/*
* describe callback
*/
void setSpeedCallback(const beaglebone::SpeedLR::ConstPtr& msg){
char bufferLeft[6];
bufferLeft[0] = COMMAND_WRITE;
bufferLeft[1] = COMMAND_SET_VELOCITY;     // set setpoint
can_get();

// fill buffer with msg.speedLeft
printf("%f %f \n", msg->speedLeft, msg->speedRight);
converter.Float = msg->speedLeft;
bufferLeft[2] = ((converter.Int >> 0) & 0xff);
bufferLeft[3] = ((converter.Int >> 8) & 0xff);
bufferLeft[4] = ((converter.Int >> 16) & 0xff);
bufferLeft[5] = ((converter.Int >> 24) & 0xff);

CAN_driver->write_CAN(bufferLeft, 6, MCLEFT);                       // write bufferLeft (length 5) with id 0x50

char bufferRight[6];
bufferRight[0] = COMMAND_WRITE;
bufferRight[1] = COMMAND_SET_VELOCITY;  // set setpoint

//fill buffer with msg.speedRight
converter.Float = msg->speedRight;
bufferRight[2] = ((converter.Int >> 0) & 0xff);
bufferRight[3] = ((converter.Int >> 8) & 0xff);
bufferRight[4] = ((converter.Int >> 16) & 0xff);
bufferRight[5] = ((converter.Int >> 24) & 0xff);

CAN_driver->write_CAN(bufferRight, 6, MCRIGHT);                       // write bufferRight (length 5) with id 0x51
}

/*
* describe service
*
*/


/*
* describe function
*/

bool ask_speed(char RL){ //ASK FOR SPEED
char buf[2] = {COMMAND_READ, COMMAND_GET_VELOCITY};
if (RL==MCLEFT){
CAN_driver->write_CAN(buf, 2, MCRIGHT);
return true;
}
else if (RL==MCRIGHT){
CAN_driver->write_CAN(buf, 2, MCLEFT);
return true;
}

return false;
}




bool getSpeed(char read_buffer[8], int length){	    //get speed from CAN
float speedleft = 0.0 , speedright = 0.0;
/*
char buf[2] = {COMMAND_READ, COMMAND_GET_VELOCITY};
CAN_driver->write_CAN(buf, 2, MCRIGHT);                               // send read Velocity request to Motor Controller right (0x51)
*/
//   char read_buffer[8];
char int_buf[4];
//   int length = 0,cid=0;

//   if(CAN_driver->read_CAN(read_buffer, length, cid)){
if(length >= 5){
int_buf[0] = read_buffer[1];
int_buf[1] = read_buffer[2];
int_buf[2] = read_buffer[3];
int_buf[3] = read_buffer[4];
converter.Int = ((int_buf[0] << 0) + (int_buf[1] << 8) + (int_buf[2] << 16) + (int_buf[3] << 24));
if(read_buffer[0] == MCRIGHT){
speedright = converter.Float;
}
else if(read_buffer[0] == MCLEFT) {
speedleft = converter.Float;
}
else{
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: read error";
Error_srv.request.aditionalInfo = "Motor Controller read";
Error_client.call(Error_srv);
printf("Data error Motor Controller R\n");
}
}
else if(length < 0){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: Timeout";
Error_srv.request.aditionalInfo = "Motor Controller right";
Error_client.call(Error_srv);
printf("Couldn't read \n");
}
else{
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: read error";
Error_srv.request.aditionalInfo = "Motor Controller right";
Error_client.call(Error_srv);
printf("Couldn't read length \n");
}

/*
// CAN_driver->write_CAN(buf, 2, MCLEFT);                               // send read Velocity request to Motor Controller right (0x50)

//if(CAN_driver->read_CAN(read_buffer, length, cid)){
//printf("received can_id %X \r\n",cid );
if((length >= 5){
int_buf[0] = read_buffer[1];
int_buf[1] = read_buffer[2];
int_buf[2] = read_buffer[3];
int_buf[3] = read_buffer[4];
converter.Int = ((int_buf[0] << 0) + (int_buf[1] << 8) + (int_buf[2] << 16) + (int_buf[3] << 24));
if(read_buffer[0] == MCLEFT){
speedleft = converter.Float;
} else {
speedright = converter.Float;
}
}
else {
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: read error";
Error_srv.request.aditionalInfo = "Motor Controller left";
Error_client.call(Error_srv);
printf("Couldn't read length \n");

}
// }
else (length < 0){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: Timeout";
Error_srv.request.aditionalInfo = "Motor Controller left";
Error_client.call(Error_srv);
printf("Couldn't read \n");
}*/

//publish it
beaglebone::SpeedLR msg;		//make message msg
msg.speedLeft = speedleft;		//fill message
msg.speedRight = speedright;
get_speed_pub.publish(msg);		//publish
return true;
}



void can_get(){
char read_buffer[8];
int length = 0,cid=0;

if(CAN_driver->read_CAN(read_buffer, length, cid)){
//printf("received can_id %X \r\n",cid );

// Create a CAN ros message:
// TODOros_can_msg = new ??

// publush can message on topic:
//	  this.can_message_topic.publish(ros_can_msg);

switch (cid)
{
case BEAGLEBONE:
getSpeed(read_buffer,length); 		//get speed from CAN
break;

case ANS_LRSI: case ANS_LRSR:
getLRS(read_buffer, length, cid);			//get LRS from CAN
break;

case ANS_GYR: case ANS_ACC: case ANS_AKM: case ANS_TEMPR: // get IMU from CAN
getIMU(read_buffer, length, cid);
break;

default:
printf("wrong can_id %X \r\n",cid );
break;
}
}
}

void ask_IMU(char sens){
char buf[1] = {0x00};
switch (sens)
{
case REQ_TEMPR:
CAN_driver->write_CAN(buf, 0, REQ_TEMPR);	// req TEMR
break;

case REQ_ACC:
CAN_driver->write_CAN(buf, 0,REQ_ACC ); 	// req ACC data
break;

case REQ_AKM:
CAN_driver->write_CAN(buf, 0,REQ_AKM );		//req AKM
break;

case REQ_GYR:
CAN_driver->write_CAN(buf, 0,REQ_GYR );		//req GYRO
break;

default:
break;
}
}

void getIMU(char read_buffer[8],int length ,int cid){
float temp = 0.0;
float Gdata[3];
float Adata[3];
float Kdata[3];


if(cid==ANS_TEMPR){				  // chek resp address 0x64 and data length
temp = create_float_from_bytes(read_buffer, 0);     // conf char to float
temp = ((temp/340)+36.53);					  // temperature convertion to Celcius and store data
}



if(cid==ANS_GYR){	 			   // chek resp address 0x58 and data length
Gdata[0] = create_float_from_bytes(read_buffer, 0);
Gdata[1] = create_float_from_bytes(read_buffer, 2);
Gdata[2] = create_float_from_bytes(read_buffer, 4);
}

if(cid==ANS_ACC){						  // chek resp address 0x60 and data length
Adata[0] = create_float_from_bytes(read_buffer, 0);
Adata[1] = create_float_from_bytes(read_buffer, 2);
Adata[2] = create_float_from_bytes(read_buffer, 4);
}

if(cid==ANS_AKM){					   // chek resp address 0x62 and data length
Kdata[0]= create_float_from_bytes(read_buffer, 0);// conf char to float and store data
Kdata[1]= create_float_from_bytes(read_buffer, 2);
Kdata[2]= create_float_from_bytes(read_buffer, 4);
}

//make message with type IMU
beaglebone::IMU IMU_msg;
IMU_msg.tmpr = temp;
IMU_msg.Xgyr = Gdata[0];
IMU_msg.Ygyr = Gdata[1];
IMU_msg.Zgyr = Gdata[2];


IMU_msg.Xacc = Adata[0];
IMU_msg.Yacc = Adata[1];
IMU_msg.Zacc = Adata[2];
/*
IMU_msg.Xkom = Kdata[0];
IMU_msg.Ykom = Kdata[1];
IMU_msg.Zkom = Kdata[2];
*/
IMU_RAW_pub.publish(IMU_msg);
}


void ask_LRS(){
char buf[0];
CAN_driver->write_CAN(buf, 0, REQ_LRS);
}


void getLRS(char read_bluffer[8],int length,int cid){

if(cid ==ANS_LRSR){						//This is a range message
LRS_msg.IdxR = read_bluffer[0];
LRS_msg.ran1 = (float(read_bluffer[2]<<8|read_bluffer[3]))/1000;
LRS_msg.ran2 = (float(read_bluffer[4]<<8|read_bluffer[5]))/1000;
LRS_msg.ran3 = (float(read_bluffer[6]<<8|read_bluffer[7]))/1000;
LRS_ran_flag = true;
}
else if(cid ==ANS_LRSI){				//this is an intensity message
LRS_msg.IdxI = read_bluffer[0];
LRS_msg.intens1 = (float(read_bluffer[2]<<8|read_bluffer[3]));
LRS_msg.intens2 = (float(read_bluffer[4]<<8|read_bluffer[5]));
LRS_msg.intens3 = (float(read_bluffer[6]<<8|read_bluffer[7]));
LRS_int_flag = true;
}
else if(length != 8){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: LRS message Lenght incorrect";
Error_srv.request.aditionalInfo = "Sensorboard";
Error_client.call(Error_srv);
printf("LRS Wrong lenght CAN message \n");
}
else{
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "CAN: LRS message Timeout";
Error_srv.request.aditionalInfo = "Sensorboard";
Error_client.call(Error_srv);
printf("Couldn't read LRS message \n");
}


if(LRS_ran_flag == true && LRS_int_flag == true){

LRS_RAW_pub.publish(LRS_msg);

LRS_msg.IdxR=0;
LRS_msg.ran1=0;
LRS_msg.ran2=0;
LRS_msg.ran3=0;
LRS_msg.IdxI=0;
LRS_msg.intens1=0;
LRS_msg.intens1=0;
LRS_msg.intens1=0;


LRS_ran_flag = false;
LRS_int_flag = false;
}
}

void errorPolling(){
//this function polls the errors of the Motor Controller. it is called from updateHook(), init() and idle()

char buffer[8];
int length = 0,cid=0;

if(CAN_driver->read_CAN(buffer, length, cid)){		//call one time the read_CAN function and check if it succeeded
if(length > 0){									//if error occured, notivy Error Handler
if(buffer[0] == 'E'){
Error_srv.request.nodeName = node_name;
Error_srv.request.errorMessage = "";
Error_srv.request.aditionalInfo = "";

if(buffer[1] == MCLEFT){
Error_srv.request.aditionalInfo = "Motor Controller left";
} else if(buffer[1] == MCRIGHT){
Error_srv.request.aditionalInfo = "Motor Controller right";
}

if(buffer[2] == ERROR_SETPOINT_BEFORE_INIT){            //error code for init setpoint before initialize
Error_srv.request.errorMessage = "Init: Setpoint before initialize";
} else if(buffer[2] == ERROR_FILTER_DIVIDE_ZERO){     //error code for filter divided by zero
Error_srv.request.errorMessage = "Filter: Divided by zero";
}
Error_client.call(Error_srv);
}
}
}
//else do nothing
}


};

int main(int argc, char **argv){
ros::init(argc, argv, "Protocol_Controller_CAN");
ros::NodeHandle n;

protocol_controller_CAN CAN(n, "protocol_controller_CAN", 100);

CAN.loop();

return 0;
}

