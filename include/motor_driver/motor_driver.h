#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__	1

// C++ library headers

#include <boost/thread.hpp>
#include <bits/stdc++.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include "canlib.h"

//ROS headers

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

class Motor{
private:
	char motor_activate_msg[24] = { 't', '0', '0', '1', '8',
			'f','f',
			'f','f',
			'f','f',  
			'f','f',
			'f','f',
			'f','f',
			'f','f',
			'f','c', '\r' };
	char motor_deactivate_msg[24] = { 't', '0', '0', '1', '8',
			'f','f',
			'f','f',
			'f','f',
			'f','f',
			'f','f',
			'f','f',
			'f','f',
			'f','d', '\r' };
    char read_buf[1000];
	char read_data[1000];
    char header[24] = {'t', '0', '0', '1', '8'};
    char cmd_data[17] = "";
    string port_name = "/dev/ttyUSB0";
    int serial_port=0;
    struct termios tty;

	float position=0, velocity=0.0, torque=0, kp=0, kd=0;

    ros::Publisher motor_state_publisher;


public:
    Motor(ros::NodeHandle *nodehandle);
	~Motor();
    void reading();
    void writing();
    void cmd_data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

};




#endif
