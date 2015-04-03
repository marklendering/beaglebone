#ifndef CANDRIVER_H
#define CANDRIVER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"

using namespace std;
namespace DemconRobot{
	class CanDriver {

		private:
			int Socket, error;                  // holds the can socket, holds return values
			int bytes_received, bytes_send;     // holds the amount of bytes received, amount of bytes send
			int timeout, temp_canId;
			struct ifreq interface;             // holds the interface (can1)
			struct sockaddr_can addr;           // holds the address of can1
			struct can_frame receive_frame;     // holds the received frame from CAN
			struct can_frame send_frame;        // holds the send frame for CAN

		public:
			CanDriver();
			bool open_Bus(char* Interface);
			bool read_Bus(char* buf, int& len,int& canid );
			bool write_Bus(char * buf, int len, int id);
	};
}

#endif
