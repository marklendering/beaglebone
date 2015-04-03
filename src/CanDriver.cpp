#include <CanDriver.h>

#include <fcntl.h>
using namespace std;

namespace DemconRobot{
	CanDriver::CanDriver()
	{
		error = 0;
		bytes_received = 0;
		bytes_send = 0;
	}

	bool CanDriver::open_Bus(char * Interface){
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

		int flags;
		flags = fcntl(Socket, F_GETFL, 0);
		fcntl(Socket, F_SETFL, flags | O_NONBLOCK);

		return true;
	}

	bool CanDriver::read_Bus(char * buf, int& len, int& canid){
		timeout = 0,temp_canId = 0;

		bytes_received = -1;

		//timeout check
		while(bytes_received < 0){
			timeout++;

			/* receive a CAN frame */
			bytes_received = read( Socket, &receive_frame, sizeof(receive_frame) );

			if(bytes_received <= 0){

				if(timeout >= 1000){
				//printf("CAN timeout \n");
				len = bytes_received;
				return false;
				}
			}
		}

		/* check if read was succesfull + paraniod check */
		if(bytes_received < 0){
			perror("error while reading can raw socket \n");
			printf("%d \n", bytes_received);
			return false;
		} else if(bytes_received < sizeof(struct can_frame)) {
			perror("read: incomplete CAN frame \n");
			return false;
		}


		len = receive_frame.can_dlc;
		temp_canId =receive_frame.can_id;

		if ((temp_canId & 0x80000000) == 0x80000000){
			canid = (temp_canId & 0x1fffffff);
		}
		else if ((temp_canId & 0x80000000) == 0x00){
			canid = (temp_canId & 0x7ff);
		}
		for (int i = 0; i<len; i++){
			buf[i] = receive_frame.data[i];
		}

	  return true;
	}

	bool CanDriver::write_Bus(char * buf, int len, int id){

		/* send a CAN frame */
		send_frame.can_id = (id & 0x7ff);                                  // fill the can ID with id + bitmask
			for(int i = 0; i<len; i++){                                     // copy the data from buf into the data of the frame
			send_frame.data[i] = buf[i];
		}
		send_frame.can_dlc = len;
		bytes_send = write( Socket, &send_frame, sizeof(send_frame) ); // send the frame into the socket

		/* check if write was succesfull + paraniod check */
		if(bytes_send < 0){
			perror("error while writing to can raw socket:\n");
			printf("%d\n", bytes_send);
			return false;
		} else if(bytes_send < sizeof(struct can_frame)) {
			perror("write: incomplete CAN frame \n");
			return false;
		}
		return true;
	}
}
