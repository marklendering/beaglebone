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

class CAN_Driver {

private:
  int Socket, error;                  // holds the can socket, holds return values
  int bytes_received, bytes_send;     // holds the amount of bytes received, amount of bytes send
  struct ifreq interface;             // holds the interface (can1)
  struct sockaddr_can addr;           // holds the address of can1
  struct can_frame receive_frame;     // holds the received frame from CAN
  struct can_frame send_frame;        // holds the send frame for CAN

public:
  CAN_Driver();
  bool open_CAN(char* Interface);
  bool read_CAN(char* buf, int& len,int& canid );
  bool write_CAN(char * buf, int len, int id);
};
