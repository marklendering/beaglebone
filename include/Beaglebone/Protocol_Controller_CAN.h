#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "Beaglebone/SpeedLR.h"
#include "Beaglebone/AppendError.h"

//include base class
#include "Beaglebone/base_node.h"
#include "Beaglebone/CAN_Driver.h"

#define MBEDLEFT  0x50
#define MBEDRIGHT 0x51
#define BEAGLEBONE 0x52
#define ALLMBEDS  0xff

// define commands

#define COMMAND_WRITE   '!'
#define COMMAND_READ    '?'

#define COMMAND_PGAIN               'p'
#define COMMAND_IGAIN               'i'
#define COMMAND_DGAIN               'd'
#define COMMAND_CONFIGURE_CONTROLLER 'c'
#define COMMAND_START_CONTROLLER    's'
#define COMMAND_STOP_CONTROLLER     't'
#define COMMAND_RESET_CONTROLLER    'r'

#define COMMAND_MODE                'm'  //mode 1 = Velocity control, mode 2 = position control (not implemented yet)

#define COMMAND_SET_VELOCITY        'v'
#define COMMAND_SET_VELOCITY_LIMIT  'u'
#define COMMAND_GET_VELOCITY        'V'

////////////////////////
// #define COMMAND_SET_ACCELERATION    'a' 
// #define COMMAND_SET_ACCELERATION_LIMIT '' 
// #define COMMAND_GET_ACCELERATION    'A'   
////////////////////////

#define COMMAND_SET_POSITION        'x'
#define COMMAND_SET_POSITION_LIMIT  'y'
#define COMMAND_GET_POSITION        'X'

#define COMMAND_STATUS_REGISTER     'S'

#define COMMAND_ERROR               'E'
#define COMMAND_FILTER_GAIN_A       'F'
#define COMMAND_FILTER_GAIN_B       'G'

#define ERROR_FILTER_DIVIDE_ZERO    3
#define ERROR_SETPOINT_BEFORE_INIT  1

union{
  float Float;
  int Int;
} converter;
