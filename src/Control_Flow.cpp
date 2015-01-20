#include "Control_Flow.h"

void control_flow::init(void){
}

void control_flow::idle(void){
}

void control_flow::run(void){
  updateHook();
}

void control_flow::error(void){
}

void control_flow::loop(void){

//ros::Rate loop_rate(period);

	while(ros::ok()){

		switch (State) {
			case INIT:
				init();
				break;

			case IDLE:
				idle();
				break;

			case RUN:
				run();
				break;

			case ERROR:
				error();
				break;
			default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

control_flow::control_flow(string name, ros::NodeHandle n, int period):
node(n), node_name(name), period(period), loop_rate(period)
{
	/* Initialize all clients of this node */
	SetPosition_client = node.serviceClient<beaglebone::SetPosition>("SetPosition");
	Error_client = node.serviceClient<beaglebone::AppendError>("AppendError");

	/* Initialize all servers of this node */
	Mode_server = node.advertiseService("Mode", &control_flow::Mode, this);
	Init_server = node.advertiseService("Init", &control_flow::Init, this);
	Command_server = node.advertiseService("Command", &control_flow::Command, this);

	/* Initialize all subscribers of this node */
	Error_sub = node.subscribe("Error", 1000, &control_flow::Error_callback, this);
	GoalVelocity_sub = node.subscribe("GoalVelocity", 1000, &control_flow::GoalVelocity_callback, this);
	GoalPosition_sub = node.subscribe("GoalPosition", 1000, &control_flow::GoalPosition_callback, this);

	/* Initialize all publishers of this node */
	OperationState_pub = node.advertise<beaglebone::OperationState>("OperationState", 1000);
	Velocity_pub = node.advertise<beaglebone::Velocity>("Velocity",1000);
	SetMode_pub = node.advertise<beaglebone::SetMode>("SetMode", 1000);


	//ros::Rate temp_rate(period);
	//loop_rate = temp_rate;

	State = INIT;
}

void control_flow::GoalVelocity_callback(const beaglebone::Velocity::ConstPtr& msg){
	if(((State == RUN) || (State == IDLE)) && (mode == VELOCITY)){
		Velocity_msg.speed = msg->speed;
		Velocity_msg.angular = msg->angular;
		Velocity_pub.publish(Velocity_msg);
	}
	else {
		Error_srv.request.nodeName = node_name;
		Error_srv.request.errorMessage = "Wrong state or wrong mode, couldn't compute Velocity";
		Error_srv.request.aditionalInfo = "";
		Error_client.call(Error_srv);
	}
}

void control_flow::GoalPosition_callback(const beaglebone::Position::ConstPtr& msg){
	if((State == IDLE) && (mode == POSITION)){
		SetPosition_srv.request.x = msg->x;
		SetPosition_srv.request.y = msg->y;
		SetPosition_srv.request.theta = msg->theta;
		if(SetPosition_client.call(SetPosition_srv)){
			//succes
		}
		else {
			//failed
			Error_srv.request.nodeName = node_name;
			Error_srv.request.errorMessage = "Failed to call SetPosition";
			Error_srv.request.aditionalInfo = "";
			Error_client.call(Error_srv);
		}
	}
	else {
		Error_srv.request.nodeName = node_name;
		Error_srv.request.errorMessage = "Wrong state or wrong mode, couldn't compute Position";
		Error_srv.request.aditionalInfo = "";
		Error_client.call(Error_srv);
	}
}

void control_flow::Error_callback(const beaglebone::Error::ConstPtr& msg){
	//ErrorState = msg->error;
	if(msg->errorState == 2){
		State = ERROR;
		OperationState_msg.state = ERROR;
		OperationState_pub.publish(OperationState_msg);
	}
	else if(msg->errorState == 1){
		if(State != INIT){
			State = IDLE;
			OperationState_msg.state = IDLE;
			OperationState_pub.publish(OperationState_msg);
		}
	}
}

bool control_flow::Command(beaglebone::Command::Request &req, beaglebone::Command::Response &res){
	int command = req.state;
	if(command == RESET){
		State = INIT;
		OperationState_msg.state = INIT;
		OperationState_pub.publish(OperationState_msg);
		return true;
	}
	else if(command == START){
			if(State == IDLE){
			State = RUN;
			OperationState_msg.state = RUN;
			OperationState_pub.publish(OperationState_msg);
			return true;
		}
		else {
			Error_srv.request.nodeName = node_name;
			Error_srv.request.errorMessage = "Start command while not in IDLE state";
			Error_srv.request.aditionalInfo = "";
			Error_client.call(Error_srv);
			return false;
		}
	}
	else if(command == STOP){
		if(State == RUN){
			State = IDLE;
			OperationState_msg.state = IDLE;
			OperationState_pub.publish(OperationState_msg);
			return true;
		}
		else {
			Error_srv.request.nodeName = node_name;
			Error_srv.request.errorMessage = "Stop command while not in RUN state";
			Error_srv.request.aditionalInfo = "";
			Error_client.call(Error_srv);
			return false;
		}
	}
	return false;
}

bool control_flow::Init(beaglebone::Init::Request &req, beaglebone::Init::Response &res){

	//init from request to params...
	node.setParam("pGain", req.Pgain);
	node.setParam("iGain", req.Igain);
	node.setParam("dGain", req.Dgain);

	node.setParam("pGainAngular", req.pGainAngular);
	node.setParam("iGainAngular", req.iGainAngular);
	node.setParam("pGainSpeed", req.pGainSpeed);

	node.setParam("wheelDiameter", req.wheelDiameter);
	node.setParam("wheelDistance", req.wheelDistance);
	node.setParam("widthCar", req.widthCar);

	node.setParam("defaultSpeed", req.defaultSpeed);
	node.setParam("defaultAngular", req.defaultAngular);

	node.setParam("speedLimit", req.speedLimit);
	node.setParam("speedAngular", req.speedAngular);

	node.setParam("verbose", req.verbose);
	node.setParam("period", req.period);

	period = req.period;
	loop_rate = ros::Rate(period);
	/* float32 Pgain
	float32 Igain
	float32 Dgain

	int32 wheelDiameter
	int32 wheelDistance
	int32 widthCar

	float32 defaultSpeed 	//(m/s)
	float32 defaultSpeed	//(rad/s)

	float32 speedLimit	//(m/s)
	float32 speedAngular	//(rad/s)

	bool verbose
	int32 period		//(Hz)*/

	if(State == INIT){
		State = IDLE;
		OperationState_msg.state = IDLE;
		OperationState_pub.publish(OperationState_msg);

		//setting the parameters

		return true;
	}
	else {
		Error_srv.request.nodeName = node_name;
		Error_srv.request.errorMessage = "Init command while not in INIT state";
		Error_srv.request.aditionalInfo = "";
		Error_client.call(Error_srv);
		return false;
	}
}

bool control_flow::Mode(beaglebone::Mode::Request &req, beaglebone::Mode::Response &res){
	if((State == INIT) || (State == IDLE)){
		mode = req.mode;
		SetMode_msg.mode = mode;
		SetMode_pub.publish(SetMode_msg);
		return true;
	}
	else {
		Error_srv.request.nodeName = node_name;
		Error_srv.request.errorMessage = "Mode command while not in INIT or IDLE state";
		Error_srv.request.aditionalInfo = "";
		Error_client.call(Error_srv);
		return false;
	}
}

void control_flow::updateHook(){
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "Control_Flow");
	ros::NodeHandle n;

	control_flow control("control_flow", n, 30);

	control.loop();

	return 0;
}
