#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Char.h" // hier moet eigen message

using namespace std;

class CAN_controll{

private:
  int Socket, error;                  // holds the can socket, holds return values
  int bytes_received, bytes_send;     // holds the amount of bytes received, amount of bytes send
  struct ifreq interface;             // holds the interface (can1)
  struct sockaddr_can addr;           // holds the address of can1
  struct can_frame receive_frame;     // holds the received frame from CAN
  struct can_frame send_frame;        // holds the send frame for CAN
  ros::NodeHandle _n;
  ros::Subscriber subSetSpeed;

public:
  CAN_controll(ros::NodeHandle n):
    _n(n)
  {
    error = 0;
    bytes_received = 0;
    bytes_send = 0;
    subSetSpeed = _n.subscribe("setSpeed", 1000, &CAN_controll::setSpeedCallback, this);
  }

  bool open_CAN(char * Interface){
    // create the socket with the raw protocol
    Socket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
    
    // check if valid socket
    if(Socket < 0) {
      perror ("error no socket ");
      return false;
    }

    /* Locate the interface you wish to use */
    strcpy(interface.ifr_name, Interface);     // copy string can0 into ifr_name
    error = ioctl(Socket, SIOCGIFINDEX, &interface);   /* interface.ifr_ifindex gets filled with that device's index */

    /* check if ioctl was succesfull */
    if(error < 0){
      perror("error filling interface.ifr_ifindex \n");
      return false;
    }

    /* Select that CAN interface, and bind the socket to it. */
    addr.can_family = AF_CAN;                                    // copy address of the CAN family
    addr.can_ifindex = interface.ifr_ifindex;                    // copy the index of the interface
    error = bind( Socket, (struct sockaddr*)&addr, sizeof(addr) );  // bind the address with the socket

    /* check if bind was succesfull */
    if(error < 0){
      perror("error while binding the CAN interface into the socket \n");
      return false;
    }

    return true;
  }

  bool read_CAN(char * buf, int& len, int& id){
    int i;
    
    /* receive a CAN frame */
    bytes_received = read( Socket, &receive_frame, sizeof(receive_frame) );

    /* check if read was succesfull + paraniod check */
    if(bytes_received < 0){
      perror("erro while reading can raw socket \n");
      return false;
    } else if(bytes_received < sizeof(struct can_frame)) {
      perror("read: incomplete CAN frame \n");
      return false;
    }
    
    len = receive_frame.can_dlc;
    id = receive_frame.can_id;

    for (i = 0; i<len; i++){
      buf[i] = receive_frame.data[i];
    }
    
    return true;
  }

  bool write_CAN(char * buf, int len, int id){
    int i;

    /* send a CAN frame */
    send_frame.can_id = (id & 0x7ff);                                  // fill the can ID with id + bitmask
    for(i = 0; i<len; i++){                                     // copy the data from buf into the data of the frame
      send_frame.data[i] = buf[i];
    }
    send_frame.can_dlc = len;
    bytes_send = write( Socket, &send_frame, sizeof(send_frame) ); // send the frame into the socket

    /* check if write was succesfull + paraniod check */
    if(bytes_send < 0){
      perror("erro while writing to can raw socket \n");
      return false;
    } else if(bytes_send < sizeof(struct can_frame)) {
      perror("write: incomplete CAN frame \n");
      return false;
    }
    return true;
  }

  void setSpeedCallback(const std_msgs::Char::ConstPtr& msg){
    char buffer[5];
    buffer[0] = 's';                                  // set setpoint

    // fill buffer with msg.data
    buffer[1] = msg->data;

    write_CAN(buffer, 2, 0x50);                       // write buffer (length 2) with id 0x50
  }

  //bool getSpeed(){
  //}

};

int main(int argc, char **argv){
  ros::init(argc, argv, "listener_can");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);

  char data[8]; 
  char data_send[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  char data_send2[8] = {0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89};
  char data_send3[8] = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87};
  int length = 8, count = 0, i, id = 0;

  CAN_controll CAN(n);

  if(!CAN.open_CAN("can0")){
    perror("Couldn't open the CAN interface");
  }

  //send first data set
  if(!CAN.write_CAN(data_send, 8, 0x50))
    printf("failed to write count");

  //receive first data set
  if(!CAN.read_CAN(data, length, id))
    printf("Couldn't read a frame... \n");
  else {
    printf("<%x> [%d] ", id, length);
    for(i = 0; i<length; i++){
      printf("%x ", data[i]);
    }
    printf("\n");
  }

  //send second data set
  if(!CAN.write_CAN(data_send2, 8, 0x50))
    printf("failed to write count");

  //receive second data set
  if(!CAN.read_CAN(data, length, id))
    printf("Couldn't read a frame... \n");
  else {
    printf("<%x> [%d] ", id, length);
    for(i = 0; i<length; i++){
      printf("%x ", data[i]);
    }
    printf("\n");
  }

  //send last data set
  if(!CAN.write_CAN(data_send3, 8, 0x50))
    printf("failed to write count");

  //send last data set
  if(!CAN.read_CAN(data, length, id))
    printf("Couldn't read a frame... \n");
  else {
    printf("<%x> [%d] ", id, length);
    for(i = 0; i<length; i++){
      printf("%x ", data[i]);
    }
    printf("\n");
  }


  return 0;
}
