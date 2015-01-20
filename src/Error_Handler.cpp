#include "ros/ros.h"

#include "beaglebone/AppendError.h"		//service AppendError
#include "beaglebone/Error.h"				//topic Error

#include "base_node.h"

class error_handler : public base_node {
private:
	ros::NodeHandle _n;
	ros::Publisher error_pub;
	ros::ServiceServer append_error_srv;
	beaglebone::Error error_msg;

public:
	error_handler(ros::NodeHandle n, string node_name, int period):
	base_node(node_name, n, period), _n(n)
	{
	  error_pub = _n.advertise<beaglebone::Error>("Error", 1000);
	  append_error_srv = _n.advertiseService("AppendError", &error_handler::append_error, this);
	}

	~error_handler(){
	}

	bool append_error(beaglebone::AppendError::Request &req, beaglebone::AppendError::Response &res ){
		//van wie komt het bericht?

		//switch (req.nodeName) {
		//case "control_flow":                               //alle control_flow errors doorsturen
		if(req.nodeName == "control_flow"){
			error_msg.nodeName = req.nodeName;
			error_msg.errorMessage = req.errorMessage;
			error_msg.aditionalInfo = req.aditionalInfo;
			error_msg.errorState = 0;                        //stay in the current state
			error_pub.publish(error_msg);
			//break;
		}

		  //case "motion_planner":

		else if(req.nodeName == "motion_planner"){
			if(req.errorMessage == "No path found"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 1;                        //stay in the current state
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "No setpoint in run state"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in the current state
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "Couldn't call a stop command"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in the current state
				error_pub.publish(error_msg);
			}
			// break;
		}

		//case "controller":
		else if(req.nodeName == "controller"){

			if(req.errorMessage == "Instability"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 2;                        //stay in the current state
				error_pub.publish(error_msg);
			}
			//break;
		}

		//case "inverse_kinematics":
		else if(req.nodeName == "inverse_kinematics"){
			if(req.errorMessage == "Divided by zero"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 2;                        //go to error state to receive a reset
				error_pub.publish(error_msg);
			}
			//break;
		}

		// case "sanity_check":
		else if(req.nodeName == "sanity_check"){
			if(req.errorMessage == "Speed higher than Vmax"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //go to idle state to receive new setpoints
				error_pub.publish(error_msg);
			}
		  // break;
		}

		  //case "protocol_controller_CAN":
		else if(req.nodeName == "protocol_controller_CAN"){
			if(req.errorMessage == "Init: Setpoint before initialize"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in the current state
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "Filter: Divided by zero"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 2;                        //go to error state to receive a reset
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "CAN: Timeout"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in current state??
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "CAN: setting up network"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in current state
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "CAN: read error"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 1;                        //go to idle state
				error_pub.publish(error_msg);
			} else if(req.errorMessage == "CAN: write error"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 1;                        //go to idle state
				error_pub.publish(error_msg);
			}
			// break;
		}

		  // case "state_estimation":
		else if(req.nodeName == "state_estimation"){
			//errors from state estimation...
			// break;
		}

		  //case "module_info":
		else if(req.nodeName == "module_info"){
			if(req.errorMessage == "Bad service response from Finished"){
				error_msg.nodeName = req.nodeName;
				error_msg.errorMessage = req.errorMessage;
				error_msg.aditionalInfo = req.aditionalInfo;
				error_msg.errorState = 0;                        //stay in the current state
				error_pub.publish(error_msg);
			}
			// break;
		}

		  //default:
		else {
			//unknown error...
			error_msg.nodeName = req.nodeName;
			error_msg.errorMessage = req.errorMessage;
			error_msg.aditionalInfo = "unknown";
			error_msg.errorState = 0;                        //stay in current state
			error_pub.publish(error_msg);
			//break;
			// }
		}
		return true;
	}

	void errorHook(){
	}

	void updateHook(){
	}

	void configureHook(){
	}

	void resetHook(){
	}

	void startHook(){
	}

	void stopHook(){
	}

};

int main(int argc, char ** argv){
	ros::init(argc, argv, "Error_Handler");
	ros::NodeHandle n;

	error_handler error(n, "error_handler", 30);

	error.loop();

	return 0;
}
