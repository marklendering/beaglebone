//include the CAN driver
#include "CAN_Driver.h"

#define MCLEFT  	0x50
#define MCRIGHT 	0x51
#define BEAGLEBONE 	0x52
#define ALLMBEDS  	0xFF

#define ANS_LRSI	0x55
#define ANS_LRSR	0x56
#define REQ_LRS		0x57
#define ANS_GYR 	0x58
#define REQ_GYR		0x59
#define ANS_ACC		0x60
#define REQ_ACC 	0x61
#define ANS_AKM 	0x62
#define REQ_AKM 	0x63
#define	ANS_TEMPR	0x64
#define REQ_TEMPR	0x65


// define commands

#define COMMAND_WRITE   			'!'
#define COMMAND_READ    			'?'

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

//converter to convert a float into a int with the same bytes. nessesary to preform bit opperation on floats
union{
  float Float;
  int Int;
} converter;
