#include "core2forAWS.h"


#define ON 0
#define OFF 1

#define NUM_INPUT_PORT  6
#define MAX_NAME_LENGTH 30
#define MAX_DATE_TIME_LENGTH 64    //Format 2015-10-26T07:46:36.611Z form date() command 
#define MAX_STATUS_STRING 15

#define GPIO_INPUT_IO_0     13
#define GPIO_INPUT_IO_4     19
#define GPIO_INPUT_IO_5     36
#define GPIO_INPUT_IO_3     35
#define GPIO_INPUT_IO_2     33
#define GPIO_INPUT_IO_1     26
#define UV_LIGHT  GPIO_NUM_32
#define ALCOHOL_SPRAY GPIO_NUM_27
// Sensor port
#define FRONT_FALL_SENSOR  GPIO_INPUT_IO_1
#define FRONT_COLLISION_SENSOR  GPIO_INPUT_IO_2
#define BACK_FALL_SENSOR  GPIO_INPUT_IO_3
#define BACK_COLLISION_SENSOR  GPIO_INPUT_IO_4

// Move Direction
#define FORWARD 1 
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

// Value for Motor Turn, U Turn, Back Off
// This will vary for motor need to readjust for different motor
// Motor should have encoder for precise movement
#define TURN_LEFT 4500
#define TURN_RIGHT 4500
#define U_TURN 9000
#define CONTINUOUS 0
#define BACK_OFF 2000




void port_init ();
void play_sound (void *arg);
int  get_input_port (int i);
void start_clean(char *);
void stop_clean();
void show_status_port();

