#include "base_node.h"

void base_node::init(void){
  //printf("%s is being initialised...\n", node_name.c_str());
}

void base_node::idle(void){
  //printf("%s is idle...\n", node_name.c_str());
}

void base_node::run(void){
  updateHook();
}

void base_node::error(void){
  //printf("an error occured in %s...\n", node_name.c_str());
}

void base_node::loop(void){

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

base_node::base_node(string name, ros::NodeHandle n, int period):
  node(n), node_name(name), period(period), loop_rate(period)
{
  OperationState_sub = node.subscribe("OperationState", 1000, &base_node::OperationState_callback, this);
  Error_client = node.serviceClient<beaglebone::AppendError>("AppendError");
  State = INIT;
}

void base_node::OperationState_callback(const beaglebone::OperationState::ConstPtr& msg){
  switch (State) {
  case INIT:
    if(msg->state == IDLE){
      configureHook();
      node.getParam("period", period);
      loop_rate = ros::Rate(period);
      node.getParam("verbose", verbose);
      State = msg->state;
    }
    break;
  case IDLE:
    switch(msg->state) {
    case INIT:
      resetHook();
      State = msg->state;
      break;
    case RUN:
      startHook();
      State = msg->state;
      break;
    case ERROR:
      errorHook();
      State = msg->state;
      break;
    default:
      break;
    }

    break;
  case RUN:
    switch(msg->state) {
    case INIT:
      resetHook();
      State = msg->state;
      break;
    case IDLE:
      stopHook();
      State = msg->state;
      break;
    case ERROR:
      errorHook();
      State = msg->state;
      break;
    default:
      break;
    }

    break;
  case ERROR:

    switch(msg->state) {
    case INIT:
      resetHook();
      State = msg->state;
      break;
    case IDLE:
      stopHook();
      State = msg->state;
      break;
    case RUN:
      startHook();
      State = msg->state;
      break;
    default:
      break;
    }

    break;
  default:
    break;
    }

}
